#ifndef WIN_AUDIO_PRODUCER_HPP
#define WIN_AUDIO_PRODUCER_HPP

#pragma region license
/*
	BSD 3-Clause License

	Copyright (c) 2023, Alex

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice, this
	   list of conditions and the following disclaimer.

	2. Redistributions in binary form must reproduce the above copyright notice,
	   this list of conditions and the following disclaimer in the documentation
	   and/or other materials provided with the distribution.

	3. Neither the name of the copyright holder nor the names of its
	   contributors may be used to endorse or promote products derived from
	   this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
	FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
	DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
	SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
	OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma endregion

#pragma warning(disable : 4996)

#include <Windows.h>
#pragma comment(lib, "winmm.lib")

#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <list>
#include <vector>
#include <string>
#include <cmath>

#ifndef fwave_t
	#define fwave_t double
#endif

#ifndef string_t
	#include <string>
	#ifdef _UNICODE
		#define string_t std::wstring
		#define _T(x) L##x
	#else
		#define string_t std::string
		#define _T(x) x
	#endif
#endif

#ifndef W2S
	#define W2S(s) std::string(s.begin(), s.end())
#endif

namespace wap
{
	struct DeviceInfo
	{
		string_t name;
		uint32_t productID;
		uint32_t manufacturerID;
		uint32_t driverVersion;
	};

	void GetOutputDevices(std::vector<DeviceInfo>& devices);
	void GetInputDevices(std::vector<DeviceInfo>& devices);

	std::vector<DeviceInfo> GetOutputDevices();
	std::vector<DeviceInfo> GetInputDevices();

	template <class T = int16_t>
	class Producer
	{
	public:
		Producer(uint32_t sampleRate = 44100, uint32_t channels = 1, uint32_t blocks = 8, uint32_t blockSamples = 512);
		~Producer();

	public:
		struct AudioSample
		{
		public:
			AudioSample() = default;
			AudioSample(const string_t& wavFile, Producer& producer);

			bool Load(const string_t& wavFile, Producer& producer);
			bool Save(const string_t& wavFile, Producer& producer);

		public:
			WAVEFORMATEX wavHeader;

			fwave_t* sample = nullptr;

			uint32_t samples = 0;
			uint32_t channels = 0;

			bool sampleValid = false;

		};

		struct PlayingSample
		{
			uint32_t audioSampleID = 0;
			uint32_t samplePosition = 0;

			bool finished = false;
			bool loop = false;
		};

		std::vector<AudioSample> audioSamples;
		std::list<PlayingSample> activeSamples;

		fwave_t maxSample;

	private:
		std::atomic<fwave_t> m_GlobalTime = 0.0;

		fwave_t (*m_UserSoundSample)(uint32_t, fwave_t, fwave_t) = nullptr;
		fwave_t (*m_UserSoundFilter)(uint32_t, fwave_t, fwave_t) = nullptr;

		T* m_BlockMemory = nullptr;
		HWAVEOUT m_Device = nullptr;

		uint32_t m_SampleRate;
		uint32_t m_Channels;
		uint32_t m_BlockCount;
		uint32_t m_BlockSamples;
		uint32_t m_BlockCurrent;
		WAVEHDR* m_WaveHeaders;

		std::thread m_AudioThread;
		std::atomic<bool> m_AudioThreadActive;
		std::atomic<uint32_t> m_BlockFree;
		std::condition_variable m_IsBlockNotZero;
		std::mutex m_BlockNotZero;

	public:
		bool CreateAudio(uint32_t sampleRate = 44100, uint32_t channels = 1, uint32_t blocks = 8, uint32_t blockSamples = 512);
		bool DestroyAudio();

		fwave_t GetTime() const;

		uint32_t GetSampleRate() const;
		uint32_t GetChannels() const;
		uint32_t GetBlocks() const;
		uint32_t GetBlockSamples() const;

		// 16-bit WAVE files ONLY
		uint32_t LoadAudioSample(const string_t& wavFile);

		void PlaySample(uint32_t id, bool loop = false);
		void StopSample(uint32_t id);

		void SetUserSoundSample(fwave_t (*func)(uint32_t, fwave_t, fwave_t));
		void SetUserSoundFilter(fwave_t (*func)(uint32_t, fwave_t, fwave_t));

		fwave_t GetMixerOutput(uint32_t channel, fwave_t globalTime, fwave_t timeStep);

		void WaveOutProc(HWAVEOUT waveOut, UINT msg, DWORD param1, DWORD param2);
		static void CALLBACK WaveOutProcWrap(HWAVEOUT waveOut, UINT msg, DWORD instance, DWORD param1, DWORD param2);

		void AudioThread();
	};

#ifdef WAP_IMPL
#undef WAP_IMPL

	template <class T>
	wap::Producer<T>::AudioSample::AudioSample(const string_t& wavFile, Producer& producer)
	{
		sampleValid = Load(wavFile, producer);
	}

	template <class T>
	bool wap::Producer<T>::AudioSample::Load(const string_t& wavFile, Producer& producer)
	{
#ifdef _UNICODE
		FILE* f = fopen(W2S(wavFile).c_str(), "rb");
#else
		FILE* f = fopen(wavFile.c_str(), "rb");
#endif

		if (!f) return false;

		char dump[4];
		fread(&dump, sizeof(char), 4, f);
		if (strncmp(dump, "RIFF", 4) != 0) return false;
		fread(&dump, sizeof(char), 4, f); // Not used
		fread(&dump, sizeof(char), 4, f);
		if (strncmp(dump, "WAVE", 4) != 0) return false;

		// Read Wave description chunk
		fread(&dump, sizeof(char), 4, f); // Read "fmt "
		fread(&dump, sizeof(char), 4, f); // Not used
		fread(&wavHeader, sizeof(WAVEFORMATEX) - 2, 1, f); // Read Wave Format Structure chunk
		// Note the -2, because the structure has 2 bytes to indicate its own size
		// which are not in the wav file

		// Check if it's 16-bit WAVE file
		if (wavHeader.wBitsPerSample != 16)
		{
			fclose(f);
			return false;
		}

		// Search for audio data chunk
		int32_t chunkSize = 0;
		fread(&dump, sizeof(int8_t), 4, f); // Read chunk header
		fread(&chunkSize, sizeof(int32_t), 1, f); // Read chunk size

		while (strncmp(dump, "data", 4) != 0)
		{
			// Not audio data, so just skip it
			fseek(f, chunkSize, SEEK_CUR);
			fread(&dump, sizeof(int8_t), 4, f);
			fread(&chunkSize, sizeof(int32_t), 1, f);
		}

		samples = uint32_t(chunkSize / int32_t(wavHeader.nChannels * (wavHeader.wBitsPerSample / 8)));
		channels = (uint32_t)wavHeader.nChannels;

		sample = new fwave_t[samples * channels];

		fwave_t* sampleStream = sample;

		for (uint32_t i = 0; i < samples; i++)
		{
			for (uint32_t c = 0; c < channels; c++)
			{
				T s = 0;
				fread(&s, sizeof(T), 1, f);
				*sampleStream = (fwave_t)s / producer.maxSample;
				sampleStream++;
			}
		}

		fclose(f);
		return true;
	}

	template <class T>
	bool wap::Producer<T>::AudioSample::Save(const string_t& wavFile, Producer& producer)
	{
#ifdef _UNICODE
		FILE* f = fopen(W2S(wavFile).c_str(), "wb");
#else
		FILE* f = fopen(wavFile.c_str(), "wb");
#endif

		if (!f) return false;

		assert(0 && "TODO: Saving WAVE file");
		return true;
	}

	void wap::GetOutputDevices(std::vector<DeviceInfo>& devices)
	{
		uint32_t deviceCount = waveOutGetNumDevs();

		for (uint32_t n = 0; n < deviceCount; n++)
		{
			WAVEOUTCAPS woc;

			if (waveOutGetDevCaps(n, &woc, sizeof(WAVEOUTCAPS)) == S_OK)
			{
				DeviceInfo di;
				di.name = woc.szPname;
				di.productID = woc.wPid;
				di.manufacturerID = woc.wMid;
				di.driverVersion = woc.vDriverVersion;
				devices.push_back(di);
			}
		}
	}

	void wap::GetInputDevices(std::vector<DeviceInfo>& devices)
	{
		uint32_t deviceCount = waveInGetNumDevs();

		for (uint32_t n = 0; n < deviceCount; n++)
		{
			WAVEINCAPS woc;

			if (waveInGetDevCaps(n, &woc, sizeof(WAVEINCAPS)) == S_OK)
			{
				DeviceInfo di;
				di.name = woc.szPname;
				di.productID = woc.wPid;
				di.manufacturerID = woc.wMid;
				di.driverVersion = woc.vDriverVersion;
				devices.push_back(di);
			}
		}
	}

	std::vector<DeviceInfo> wap::GetOutputDevices()
	{
		std::vector<DeviceInfo> devices;
		GetOutputDevices(devices);
		return devices;
	}

	std::vector<DeviceInfo> wap::GetInputDevices()
	{
		std::vector<DeviceInfo> devices;
		GetInputDevices(devices);
		return devices;
	}

	template <class T>
	wap::Producer<T>::Producer(uint32_t sampleRate, uint32_t channels, uint32_t blocks, uint32_t blockSamples)
	{
		maxSample = (fwave_t)pow(2, sizeof(T) * 8 - 1) - 1.0;
		m_AudioThreadActive = CreateAudio(sampleRate, channels, blocks, blockSamples);
	}

	template <class T>
	wap::Producer<T>::~Producer()
	{
		DestroyAudio();
	}

	template <class T>
	bool Producer<T>::CreateAudio(uint32_t sampleRate, uint32_t channels, uint32_t blocks, uint32_t blockSamples)
	{
		m_SampleRate = sampleRate;
		m_Channels = channels;
		m_BlockCount = blocks;
		m_BlockSamples = blockSamples;
		m_BlockFree = m_BlockCount;
		m_BlockCurrent = 0;

		WAVEFORMATEX waveFormat;
		waveFormat.wFormatTag = WAVE_FORMAT_PCM;
		waveFormat.nSamplesPerSec = m_SampleRate;
		waveFormat.wBitsPerSample = sizeof(T) * 8;
		waveFormat.nChannels = m_Channels;
		waveFormat.nBlockAlign = (waveFormat.wBitsPerSample / 8) * waveFormat.nChannels;
		waveFormat.nAvgBytesPerSec = waveFormat.nSamplesPerSec * waveFormat.nBlockAlign;
		waveFormat.cbSize = sizeof(WAVEFORMATEX);

		if (waveOutOpen(&m_Device, WAVE_MAPPER, &waveFormat, (DWORD_PTR)WaveOutProcWrap, (DWORD_PTR)this, CALLBACK_FUNCTION) != S_OK)
			return DestroyAudio();

		m_BlockMemory = new T[m_BlockCount * m_BlockSamples];
		if (!m_BlockMemory) return DestroyAudio();

		ZeroMemory(m_BlockMemory, sizeof(T) * m_BlockCount * m_BlockSamples);

		m_WaveHeaders = new WAVEHDR[m_BlockCount];
		if (!m_WaveHeaders) return DestroyAudio();

		ZeroMemory(m_WaveHeaders, sizeof(WAVEHDR) * m_BlockCount);

		for (uint32_t n = 0; n < m_BlockCount; n++)
		{
			m_WaveHeaders[n].dwBufferLength = m_BlockSamples * sizeof(T);
			m_WaveHeaders[n].lpData = LPSTR(m_BlockMemory + n * m_BlockSamples);
		}

		m_AudioThreadActive = true;
		m_AudioThread = std::thread(&Producer::AudioThread, this);

		std::unique_lock<std::mutex> lock(m_BlockNotZero);
		m_IsBlockNotZero.notify_one();

		return true;
	}

	template <class T>
	bool Producer<T>::DestroyAudio()
	{
		m_AudioThreadActive = false;

		if (m_AudioThread.joinable())
			m_AudioThread.join();

		return false;
	}

	template <class T>
	fwave_t Producer<T>::GetTime() const
	{
		return (fwave_t)m_GlobalTime;
	}

	template <class T>
	uint32_t Producer<T>::GetSampleRate() const
	{
		return m_SampleRate;
	}

	template <class T>
	uint32_t Producer<T>::GetChannels() const
	{
		return m_Channels;
	}

	template <class T>
	uint32_t Producer<T>::GetBlocks() const
	{
		return m_BlockCount;
	}

	template <class T>
	uint32_t Producer<T>::GetBlockSamples() const
	{
		return m_BlockSamples;
	}

	template <class T>
	uint32_t wap::Producer<T>::LoadAudioSample(const string_t& wavFile)
	{
		if (!m_AudioThreadActive)
			return 0;

		AudioSample a(wavFile, *this);
		if (a.sampleValid)
		{
			audioSamples.push_back(a);
			return audioSamples.size();
		}

		return 0;
	}

	template <class T>
	void wap::Producer<T>::PlaySample(uint32_t id, bool loop)
	{
		PlayingSample s;
		s.audioSampleID = id;
		s.samplePosition = 0;
		s.finished = false;
		s.loop = loop;
		activeSamples.push_back(s);
	}

	template <class T>
	void wap::Producer<T>::StopSample(uint32_t id)
	{
		activeSamples.remove_if([id](PlayingSample& s) { return s.audioSampleID == id; });
	}

	template <class T>
	void wap::Producer<T>::SetUserSoundSample(fwave_t (*func)(uint32_t, fwave_t, fwave_t))
	{
		m_UserSoundSample = func;
	}

	template <class T>
	void wap::Producer<T>::SetUserSoundFilter(fwave_t (*func)(uint32_t, fwave_t, fwave_t))
	{
		m_UserSoundFilter = func;
	}

	template <class T>
	fwave_t wap::Producer<T>::GetMixerOutput(uint32_t channel, fwave_t globalTime, fwave_t timeStep)
	{
		fwave_t mixerSample = 0.0;

		for (auto& s : activeSamples)
		{
			fwave_t freq = audioSamples[s.audioSampleID - 1].wavHeader.nSamplesPerSec;
			s.samplePosition += uint32_t(freq * timeStep);

			if (s.samplePosition < audioSamples[s.audioSampleID - 1].samples)
			{
				uint32_t channels = audioSamples[s.audioSampleID - 1].channels;
				mixerSample += audioSamples[s.audioSampleID - 1].sample[s.samplePosition * channels + channel];
			}
			else
				s.finished = true;
		}

		activeSamples.remove_if([&](const PlayingSample& s)
			{
				if (s.finished && s.loop) PlaySample(s.audioSampleID, s.loop);
				return s.finished;
			});

		if (m_UserSoundSample)
			mixerSample += m_UserSoundSample(channel, globalTime, timeStep);

		if (m_UserSoundFilter)
			return m_UserSoundFilter(channel, globalTime, mixerSample);

		return mixerSample;
	}

	template<class T>
	void Producer<T>::WaveOutProc(HWAVEOUT waveOut, UINT msg, DWORD param1, DWORD param2)
	{
		if (msg != WOM_DONE) return;

		m_BlockFree++;

		std::unique_lock<std::mutex> lock(m_BlockNotZero);
		m_IsBlockNotZero.notify_one();
	}

	template<class T>
	void Producer<T>::WaveOutProcWrap(HWAVEOUT waveOut, UINT msg, DWORD instance, DWORD param1, DWORD param2)
	{
		((Producer*)instance)->WaveOutProc(waveOut, msg, param1, param2);
	}

	template<class T>
	void Producer<T>::AudioThread()
	{
		m_GlobalTime = 0.0;
		fwave_t timeStep = 1.0 / (fwave_t)m_SampleRate;

		while (m_AudioThreadActive)
		{
			if (m_BlockFree == 0)
			{
				std::unique_lock<std::mutex> lock(m_BlockNotZero);

				while (m_BlockFree == 0)
					m_IsBlockNotZero.wait(lock);
			}

			m_BlockFree--;

			if (m_WaveHeaders[m_BlockCurrent].dwFlags & WHDR_PREPARED)
				waveOutUnprepareHeader(m_Device, &m_WaveHeaders[m_BlockCurrent], sizeof(WAVEHDR));

			auto Clip = [](fwave_t sample, fwave_t max)
			{
				return (sample >= 0.0) ?
					fmin(sample, max) :
					fmax(sample, -max);
			};

			uint32_t currentBlock = m_BlockCurrent * m_BlockSamples;

			for (uint32_t n = 0; n < m_BlockSamples; n += m_Channels)
			{
				for (uint32_t c = 0; c < m_Channels; c++)
				{
					T newSample = T(Clip(GetMixerOutput(c, m_GlobalTime, timeStep), 1.0) * maxSample);
					m_BlockMemory[currentBlock + n + c] = newSample;
				}

				m_GlobalTime = m_GlobalTime + timeStep;
			}

			waveOutPrepareHeader(m_Device, &m_WaveHeaders[m_BlockCurrent], sizeof(WAVEHDR));
			waveOutWrite(m_Device, &m_WaveHeaders[m_BlockCurrent], sizeof(WAVEHDR));

			++m_BlockCurrent %= m_BlockCount;
		}
	}

#endif
}

#endif

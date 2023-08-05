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

#ifndef FTYPE
#define FTYPE double
#endif

#ifndef W2S
#define W2S(s) std::string(s.begin(), s.end())
#endif

namespace wap
{
	struct DeviceInfo
	{
		std::wstring sName;
		uint32_t nProductID;
		uint32_t nManufacturerID;
		uint32_t nDriverVersion;
	};

	void GetOutputDevices(std::vector<DeviceInfo>& vecDevices);
	void GetInputDevices(std::vector<DeviceInfo>& vecDevices);

	std::vector<DeviceInfo> GetOutputDevices();
	std::vector<DeviceInfo> GetInputDevices();

	template <class T = int16_t>
	class Producer
	{
	public:
		Producer(uint32_t nSampleRate = 44100, uint32_t nChannels = 1, uint32_t nBlocks = 8, uint32_t nBlockSamples = 512);
		~Producer();

	public:
		struct AudioSample
		{
		public:
			AudioSample() = default;
			AudioSample(const std::wstring& sWavFile, Producer& producer);

			void Load(const std::wstring& sWavFile, Producer& producer);

		public:
			WAVEFORMATEX wavHeader;

			FTYPE* dSample = nullptr;

			uint32_t nSamples = 0;
			uint32_t nChannels = 0;

			bool bSampleValid = false;

		};

		struct PlayingSample
		{
			uint32_t nAudioSampleID = 0;
			uint32_t nSamplePosition = 0;
			bool bFinished = false;
			bool bLoop = false;
		};

		std::vector<AudioSample> vecAudioSamples;
		std::list<PlayingSample> listActiveSamples;

		FTYPE dMaxSample;

	private:
		std::atomic<FTYPE> m_dGlobalTime = 0.0;

		FTYPE (*m_funcUserSoundSample)(uint32_t, FTYPE, FTYPE) = nullptr;
		FTYPE (*m_funcUserSoundFilter)(uint32_t, FTYPE, FTYPE) = nullptr;

		T* m_pBlockMemory = nullptr;
		HWAVEOUT m_hwDevice = nullptr;

		uint32_t m_nSampleRate;
		uint32_t m_nChannels;
		uint32_t m_nBlockCount;
		uint32_t m_nBlockSamples;
		uint32_t m_nBlockCurrent;
		WAVEHDR* m_pWaveHeaders;

		std::thread m_thrAudio;
		std::atomic<bool> m_bAudioThreadActive;
		std::atomic<uint32_t> m_nBlockFree;
		std::condition_variable m_cvBlockNotZero;
		std::mutex m_muxBlockNotZero;

	public:
		bool CreateAudio(uint32_t nSampleRate = 44100, uint32_t nChannels = 1, uint32_t nBlocks = 8, uint32_t nBlockSamples = 512);
		bool DestroyAudio();

		FTYPE GetTime() const;

		// 16-bit WAVE files ONLY
		uint32_t LoadAudioSample(const std::wstring& sWavFile);

		void PlaySample(uint32_t nID, bool bLoop = false);
		void StopSample(uint32_t nID);

		void SetUserSoundSample(FTYPE (*func)(uint32_t, FTYPE, FTYPE));
		void SetUserSoundFilter(FTYPE (*func)(uint32_t, FTYPE, FTYPE));

		FTYPE GetMixerOutput(uint32_t nChannel, FTYPE dGlobalTime, FTYPE dTimeStep);

		void WaveOutProc(HWAVEOUT hWaveOut, UINT uMsg, DWORD dwParam1, DWORD dwParam2);
		static void CALLBACK WaveOutProcWrap(HWAVEOUT hWaveOut, UINT uMsg, DWORD dwInstance, DWORD dwParam1, DWORD dwParam2);

		void AudioThread();
	};

#ifdef WAP_IMPL
#undef WAP_IMPL

	template <class T>
	wap::Producer<T>::AudioSample::AudioSample(const std::wstring& sWavFile, Producer& producer)
	{
		Load(sWavFile, producer);
	}

	template <class T>
	void wap::Producer<T>::AudioSample::Load(const std::wstring& sWavFile, Producer& producer)
	{
		FILE* f = fopen(W2S(sWavFile).c_str(), "rb");
		if (!f)
		{
			bSampleValid = false;
			return;
		}

		char dump[4];
		fread(&dump, sizeof(char), 4, f);
		if (strncmp(dump, "RIFF", 4) != 0) return;
		fread(&dump, sizeof(char), 4, f); // Not used
		fread(&dump, sizeof(char), 4, f);
		if (strncmp(dump, "WAVE", 4) != 0) return;

		// Read Wave description chunk
		fread(&dump, sizeof(char), 4, f); // Read "fmt "
		fread(&dump, sizeof(char), 4, f); // Not used
		fread(&wavHeader, sizeof(WAVEFORMATEX) - 2, 1, f); // Read Wave Format Structure chunk
		// Note the -2, because the structure has 2 bytes to indicate its own size
		// which are not in the wav file

		// Check if it's 16-bit WAVE file
		if (wavHeader.wBitsPerSample != 16)
		{
			bSampleValid = false;
			fclose(f);
			return;
		}

		// Search for audio data chunk
		int32_t nChunkSize = 0;
		fread(&dump, sizeof(int8_t), 4, f); // Read chunk header
		fread(&nChunkSize, sizeof(int32_t), 1, f); // Read chunk size

		while (strncmp(dump, "data", 4) != 0)
		{
			// Not audio data, so just skip it
			fseek(f, nChunkSize, SEEK_CUR);
			fread(&dump, sizeof(int8_t), 4, f);
			fread(&nChunkSize, sizeof(int32_t), 1, f);
		}

		nSamples = uint32_t(nChunkSize / int32_t(wavHeader.nChannels * (wavHeader.wBitsPerSample / 8)));
		nChannels = (uint32_t)wavHeader.nChannels;

		dSample = new FTYPE[nSamples * nChannels];

		FTYPE* pSample = dSample;

		for (uint32_t i = 0; i < nSamples; i++)
		{
			for (uint32_t c = 0; c < nChannels; c++)
			{
				T s = 0;
				fread(&s, sizeof(T), 1, f);
				*pSample = (FTYPE)s / producer.dMaxSample;
				pSample++;
			}
		}

		bSampleValid = true;
		fclose(f);
	}

	void wap::GetOutputDevices(std::vector<DeviceInfo>& vecDevices)
	{
		uint32_t nDeviceCount = waveOutGetNumDevs();

		for (uint32_t n = 0; n < nDeviceCount; n++)
		{
			WAVEOUTCAPS woc;

			if (waveOutGetDevCaps(n, &woc, sizeof(WAVEOUTCAPS)) == S_OK)
			{
				DeviceInfo di;
				di.sName = woc.szPname;
				di.nProductID = woc.wPid;
				di.nManufacturerID = woc.wMid;
				di.nDriverVersion = woc.vDriverVersion;
				vecDevices.push_back(di);
			}
		}
	}

	void wap::GetInputDevices(std::vector<DeviceInfo>& vecDevices)
	{
		uint32_t nDeviceCount = waveInGetNumDevs();

		for (uint32_t n = 0; n < nDeviceCount; n++)
		{
			WAVEINCAPS woc;

			if (waveInGetDevCaps(n, &woc, sizeof(WAVEINCAPS)) == S_OK)
			{
				DeviceInfo di;
				di.sName = woc.szPname;
				di.nProductID = woc.wPid;
				di.nManufacturerID = woc.wMid;
				di.nDriverVersion = woc.vDriverVersion;
				vecDevices.push_back(di);
			}
		}
	}

	std::vector<DeviceInfo> wap::GetOutputDevices()
	{
		std::vector<DeviceInfo> vecDevices;
		GetOutputDevices(vecDevices);
		return vecDevices;
	}

	std::vector<DeviceInfo> wap::GetInputDevices()
	{
		std::vector<DeviceInfo> vecDevices;
		GetInputDevices(vecDevices);
		return vecDevices;
	}

	template <class T>
	wap::Producer<T>::Producer(uint32_t nSampleRate, uint32_t nChannels, uint32_t nBlocks, uint32_t nBlockSamples)
	{
		dMaxSample = (FTYPE)pow(2, sizeof(T) * 8 - 1) - 1.0;
		m_bAudioThreadActive = CreateAudio(nSampleRate, nChannels, nBlocks, nBlockSamples);
	}

	template <class T>
	wap::Producer<T>::~Producer()
	{
	}

	template<class T>
	bool Producer<T>::CreateAudio(uint32_t nSampleRate, uint32_t nChannels, uint32_t nBlocks, uint32_t nBlockSamples)
	{
		m_nSampleRate = nSampleRate;
		m_nChannels = nChannels;
		m_nBlockCount = nBlocks;
		m_nBlockSamples = nBlockSamples;
		m_nBlockFree = m_nBlockCount;
		m_nBlockCurrent = 0;

		WAVEFORMATEX waveFormat;
		waveFormat.wFormatTag = WAVE_FORMAT_PCM;
		waveFormat.nSamplesPerSec = m_nSampleRate;
		waveFormat.wBitsPerSample = sizeof(T) * 8;
		waveFormat.nChannels = m_nChannels;
		waveFormat.nBlockAlign = (waveFormat.wBitsPerSample / 8) * waveFormat.nChannels;
		waveFormat.nAvgBytesPerSec = waveFormat.nSamplesPerSec * waveFormat.nBlockAlign;
		waveFormat.cbSize = sizeof(WAVEFORMATEX);

		if (waveOutOpen(&m_hwDevice, WAVE_MAPPER, &waveFormat, (DWORD_PTR)WaveOutProcWrap, (DWORD_PTR)this, CALLBACK_FUNCTION) != S_OK)
			return DestroyAudio();

		m_pBlockMemory = new T[m_nBlockCount * m_nBlockSamples];
		if (!m_pBlockMemory) return DestroyAudio();

		ZeroMemory(m_pBlockMemory, sizeof(T) * m_nBlockCount * m_nBlockSamples);

		m_pWaveHeaders = new WAVEHDR[m_nBlockCount];
		if (!m_pWaveHeaders) return DestroyAudio();

		ZeroMemory(m_pWaveHeaders, sizeof(WAVEHDR) * m_nBlockCount);

		for (uint32_t n = 0; n < m_nBlockCount; n++)
		{
			m_pWaveHeaders[n].dwBufferLength = m_nBlockSamples * sizeof(T);
			m_pWaveHeaders[n].lpData = LPSTR(m_pBlockMemory + n * m_nBlockSamples);
		}

		m_bAudioThreadActive = true;
		m_thrAudio = std::thread(&Producer::AudioThread, this);

		std::unique_lock<std::mutex> lock(m_muxBlockNotZero);
		m_cvBlockNotZero.notify_one();

		return true;
	}

	template<class T>
	bool Producer<T>::DestroyAudio()
	{
		m_bAudioThreadActive = false;
		if (m_thrAudio.joinable()) m_thrAudio.join();
		return false;
	}

	template<class T>
	FTYPE Producer<T>::GetTime() const
	{
		return (FTYPE)m_dGlobalTime;
	}

	template <class T>
	uint32_t wap::Producer<T>::LoadAudioSample(const std::wstring& sWavFile)
	{
		if (!m_bAudioThreadActive)
			return 0;

		AudioSample a(sWavFile, *this);
		if (a.bSampleValid)
		{
			vecAudioSamples.push_back(a);
			return vecAudioSamples.size();
		}

		return 0;
	}

	template <class T>
	void wap::Producer<T>::PlaySample(uint32_t nID, bool bLoop)
	{
		PlayingSample s;
		s.nAudioSampleID = nID;
		s.nSamplePosition = 0;
		s.bFinished = false;
		s.bLoop = bLoop;
		listActiveSamples.push_back(s);
	}

	template <class T>
	void wap::Producer<T>::StopSample(uint32_t nID)
	{
		listActiveSamples.remove_if([nID](PlayingSample& s) { return s.nAudioSampleID == nID; });
	}

	template <class T>
	void wap::Producer<T>::SetUserSoundSample(FTYPE (*func)(uint32_t, FTYPE, FTYPE))
	{
		m_funcUserSoundSample = func;
	}

	template <class T>
	void wap::Producer<T>::SetUserSoundFilter(FTYPE (*func)(uint32_t, FTYPE, FTYPE))
	{
		m_funcUserSoundFilter = func;
	}

	template <class T>
	FTYPE wap::Producer<T>::GetMixerOutput(uint32_t nChannel, FTYPE dGlobalTime, FTYPE dTimeStep)
	{
		FTYPE dMixerSample = 0.0;

		for (auto& s : listActiveSamples)
		{
			FTYPE dFreq = vecAudioSamples[s.nAudioSampleID - 1].wavHeader.nSamplesPerSec;
			s.nSamplePosition += uint32_t(dFreq * dTimeStep);

			if (s.nSamplePosition < vecAudioSamples[s.nAudioSampleID - 1].nSamples)
			{
				uint32_t nChannels = vecAudioSamples[s.nAudioSampleID - 1].nChannels;
				dMixerSample += vecAudioSamples[s.nAudioSampleID - 1].dSample[s.nSamplePosition * nChannels + nChannel];
			}
			else
				s.bFinished = true;
		}

		listActiveSamples.remove_if([&](const PlayingSample& s)
			{
				if (s.bFinished && s.bLoop) PlaySample(s.nAudioSampleID, s.bLoop);
				return s.bFinished;
			});

		if (m_funcUserSoundSample)
			dMixerSample += m_funcUserSoundSample(nChannel, dGlobalTime, dTimeStep);

		if (m_funcUserSoundFilter)
			return m_funcUserSoundFilter(nChannel, dGlobalTime, dMixerSample);

		return dMixerSample;
	}

	template<class T>
	void Producer<T>::WaveOutProc(HWAVEOUT hWaveOut, UINT uMsg, DWORD dwParam1, DWORD dwParam2)
	{
		if (uMsg != WOM_DONE) return;

		m_nBlockFree++;

		std::unique_lock<std::mutex> lock(m_muxBlockNotZero);
		m_cvBlockNotZero.notify_one();
	}

	template<class T>
	void Producer<T>::WaveOutProcWrap(HWAVEOUT hWaveOut, UINT uMsg, DWORD dwInstance, DWORD dwParam1, DWORD dwParam2)
	{
		((Producer*)dwInstance)->WaveOutProc(hWaveOut, uMsg, dwParam1, dwParam2);
	}

	template<class T>
	void Producer<T>::AudioThread()
	{
		m_dGlobalTime = 0.0;
		FTYPE dTimeStep = 1.0 / (FTYPE)m_nSampleRate;

		while (m_bAudioThreadActive)
		{
			if (m_nBlockFree == 0)
			{
				std::unique_lock<std::mutex> lock(m_muxBlockNotZero);
				while (m_nBlockFree == 0) m_cvBlockNotZero.wait(lock);
			}

			m_nBlockFree--;

			if (m_pWaveHeaders[m_nBlockCurrent].dwFlags & WHDR_PREPARED)
				waveOutUnprepareHeader(m_hwDevice, &m_pWaveHeaders[m_nBlockCurrent], sizeof(WAVEHDR));
		
			auto clip = [](FTYPE dSample, FTYPE dMax)
			{
				return (dSample >= 0.0) ?
					fmin(dSample, dMax) :
					fmax(dSample, -dMax);
			};

			uint32_t nCurrentBlock = m_nBlockCurrent * m_nBlockSamples;

			for (uint32_t n = 0; n < m_nBlockSamples; n += m_nChannels)
			{
				for (uint32_t c = 0; c < m_nChannels; c++)
				{
					T nNewSample = T(clip(GetMixerOutput(c, m_dGlobalTime, dTimeStep), 1.0) * dMaxSample);
					m_pBlockMemory[nCurrentBlock + n + c] = nNewSample;
				}

				m_dGlobalTime = m_dGlobalTime + dTimeStep;
			}

			waveOutPrepareHeader(m_hwDevice, &m_pWaveHeaders[m_nBlockCurrent], sizeof(WAVEHDR));
			waveOutWrite(m_hwDevice, &m_pWaveHeaders[m_nBlockCurrent], sizeof(WAVEHDR));

			++m_nBlockCurrent %= m_nBlockCount;
		}
	}

#endif
}

#endif

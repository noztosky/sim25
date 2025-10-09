#pragma once
#include <windows.h>
#include <string>

class x_pipeline {
private:
	HANDLE handle_;
	std::string name_;
	bool is_server_;

public:
	x_pipeline() : handle_(INVALID_HANDLE_VALUE), is_server_(false) {}
	~x_pipeline() { close(); }

	bool create_server(const char* name, DWORD openMode, DWORD pipeMode,
		DWORD outBufferSize = 65536, DWORD inBufferSize = 65536, DWORD defaultTimeout = 0) {
		close();
		name_ = name;
		is_server_ = true;
		handle_ = CreateNamedPipeA(
			name,
			openMode,
			pipeMode,
			1,
			outBufferSize,
			inBufferSize,
			defaultTimeout,
			NULL
		);
		return handle_ != INVALID_HANDLE_VALUE;
	}

	bool connect_server(BOOL* immediateResult = nullptr, DWORD* lastError = nullptr) {
		if (handle_ == INVALID_HANDLE_VALUE) return false;
		BOOL ok = ConnectNamedPipe(handle_, NULL);
		DWORD err = GetLastError();
		if (lastError) *lastError = err;
		if (immediateResult) *immediateResult = ok;
		if (ok) return true;
		if (err == ERROR_PIPE_CONNECTED) return true;
		if (err == ERROR_PIPE_LISTENING) return true;
		return false;
	}

	bool open_client(const char* name, DWORD desiredAccess, DWORD flagsAndAttributes = 0) {
		close();
		name_ = name;
		is_server_ = false;
		for (int attempt = 0; attempt < 20; ++attempt) {
			handle_ = CreateFileA(
				name,
				desiredAccess,
				0,
				NULL,
				OPEN_EXISTING,
				flagsAndAttributes,
				NULL
			);
			if (handle_ != INVALID_HANDLE_VALUE) return true;
			DWORD err = GetLastError();
			if (err == ERROR_FILE_NOT_FOUND) {
				// server not created yet; brief sleep then retry
				Sleep(50);
				continue;
			}
			if (err == ERROR_PIPE_BUSY) {
				// wait briefly for a free instance
				WaitNamedPipeA(name, 100);
				continue;
			}
			// other errors: don't hang
			break;
		}
		return false;
	}

	bool write_sync(const void* buffer, DWORD size, DWORD* bytesWritten) {
		if (handle_ == INVALID_HANDLE_VALUE) return false;
		return WriteFile(handle_, buffer, size, bytesWritten, NULL) != 0;
	}

	bool write_async(const void* buffer, DWORD size, OVERLAPPED* overlapped, DWORD* immediateBytes) {
		if (handle_ == INVALID_HANDLE_VALUE) return false;
		DWORD bytes = 0;
		BOOL ok = WriteFile(handle_, buffer, size, &bytes, overlapped);
		if (immediateBytes) *immediateBytes = bytes;
		if (ok) return true;
		DWORD err = GetLastError();
		if (err == ERROR_IO_PENDING) return true;
		return false;
	}

	bool read_sync(void* buffer, DWORD size, DWORD* bytesRead) {
		if (handle_ == INVALID_HANDLE_VALUE) return false;
		return ReadFile(handle_, buffer, size, bytesRead, NULL) != 0;
	}

	void disconnect() {
		if (is_server_ && handle_ != INVALID_HANDLE_VALUE) {
			DisconnectNamedPipe(handle_);
		}
	}

	void close() {
		if (handle_ != INVALID_HANDLE_VALUE) {
			if (is_server_) DisconnectNamedPipe(handle_);
			CloseHandle(handle_);
			handle_ = INVALID_HANDLE_VALUE;
		}
	}

	HANDLE get_handle() const {
		return handle_;
	}

	bool is_valid() const {
		return handle_ != INVALID_HANDLE_VALUE;
	}

	const std::string& name() const { return name_;
	}
};

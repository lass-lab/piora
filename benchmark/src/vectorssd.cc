#include "vectorssd.h"

#include <fcntl.h>
#include <linux/nvme_ioctl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>

#ifdef DEBUG
#include <inttypes.h>

#include <cstdio>
#endif
/*

sudo python3 pyvectorssd.py --config config.json --dataset ../../workloads/synthetic_put.csv --type a

*/
namespace {
constexpr unsigned int PAGE_SIZE = 4096;
constexpr unsigned int MAX_BUFLEN = 8 * 1024;
unsigned int NSID = 1;

} // namespace

namespace vectorssd {

DB::~DB() { Close(); }

int DB::Open(const std::string &dev, const std::string &collectionName) {
  std::cout << dev << std::endl;
  if (fd_ != -1) {
    Close();
  }

  int err = open(dev.c_str(), O_RDONLY);
  if (err < 0) {
    std::cout << "Device error." << std::endl;

    return -1;
  }

  fd_ = err;

  struct stat nvme_stat;
  err = fstat(fd_, &nvme_stat);
  if (err < 0) {
    Close();
    return -1;
  }
  if (!S_ISCHR(nvme_stat.st_mode) && !S_ISBLK(nvme_stat.st_mode)) {
    Close();
    return -1;
  }

  std::cout << "[VECTORSSD] Device has been connected." << std::endl;
  unsigned int ns_list[1024];
  struct nvme_passthru_cmd cmd = {
      .opcode = 0x06,
      .nsid = dev[dev.size()],
      .addr = (long long)ns_list,
      .data_len = 0x1000,
      .cdw10 = 2,
  };

  ioctl(fd_, NVME_IOCTL_ADMIN_CMD, &cmd);

  for (int i = 0; i < 1024; i++) {
    if (ns_list[i]) {
      NSID = ns_list[i];
      break;
    }
  }
  return 0;
}

int DB::Close() {
  if (fd_ != -1) {
    int result = close(fd_);
    fd_ = -1;
    return result;
  }
  std::cout << "[VECTORSSD] Disconnected from the device." << std::endl;
  return 0;
}

int DB::Put(const std::string &key, const float vector[VECTOR_DIMENSION], const bool delayed_compaction) {
  if (!IsOpen()) {
    return -1;
  }

  void *data = nullptr;
  unsigned int data_len = VECTOR_SIZE;
  unsigned int nlb = ((data_len - 1) / PAGE_SIZE);
  data_len = (nlb + 1) * PAGE_SIZE;
  if (posix_memalign(&data, PAGE_SIZE, data_len)) {
    return -ENOMEM;
  }
  std::memset(data, 0, data_len);
  memcpy(data, vector, VECTOR_SIZE);
  uint32_t result;
  uint32_t cdw2, cdw3, cdw10, cdw11, cdw12, cdw13, cdw14, cdw15;
  cdw2 = cdw3 = cdw10 = cdw12 = cdw13 = cdw14 = cdw15 = 0;
  cdw10 = static_cast<uint32_t>(std::stoi(key));

  cdw12 = 0 | (0xFFFF & nlb);
  cdw13 = data_len;
  int err = nvme_passthru(NVME_CMD_VECTOR_INSERT, 0, 0, NSID, cdw2, cdw3, cdw10, cdw11, cdw12, cdw13, cdw14, cdw15, data_len, data, &result);
  free(data);
  if (err < 0 || result != 0) {
    return -1;
  }

  return 0;
}

int DB::Get(const std::string &key, std::string *value) {
  if (!IsOpen()) {
    return -1;
  }
  void *data = nullptr;
  unsigned int data_len = MAX_BUFLEN;
  unsigned int nlb = (MAX_BUFLEN - 1) / PAGE_SIZE;
  if (posix_memalign(&data, PAGE_SIZE, data_len)) {
    return -ENOMEM;
  }
  std::memset(data, 0, data_len);
  uint32_t result;
  uint32_t cdw2, cdw3, cdw10, cdw11, cdw12, cdw13, cdw14, cdw15;
  cdw2 = cdw3 = cdw10 = cdw11 = cdw12 = cdw13 = cdw14 = cdw15 = 0;

  cdw10 = static_cast<uint32_t>(std::stoi(key));

  cdw12 = 0 | (0xFFFF & nlb);
  int err = nvme_passthru(NVME_CMD_KV_GET, 0, 0, NSID, cdw2, cdw3, cdw10, cdw11, cdw12, cdw13, cdw14, cdw15, data_len, data, &result);
  if (err < 0) {
    free(data);
    return -1;
  }
  if (err == 0x7C1) {
    free(data);
    return -2;
  }
  if (result > 0) {
    *value = std::string(static_cast<const char *>(data), result);
  } else {
    (*value).clear();
  }

  free(data);
  return result;
}
int DB::Print(const std::string &key) {
  if (!IsOpen()) {
    return -1;
  }
  void *data = nullptr;
  unsigned int data_len = VECTOR_SIZE;
  unsigned int nlb = ((data_len - 1) / PAGE_SIZE);
  data_len = (nlb + 1) * PAGE_SIZE;
  if (posix_memalign(&data, PAGE_SIZE, data_len)) {
    return -ENOMEM;
  }
  std::memset(data, 0, data_len);
  uint32_t result;
  uint32_t cdw2, cdw3, cdw10, cdw11, cdw12, cdw13, cdw14, cdw15;
  cdw2 = cdw3 = cdw10 = cdw12 = cdw13 = cdw14 = cdw15 = 0;
  cdw10 = static_cast<uint32_t>(std::stoi(key));
  cdw12 = 0 | (0xFFFF & nlb);
  cdw13 = data_len;
  int err = nvme_passthru(NVME_CMD_SST_PRINT, 0, 0, NSID, cdw2, cdw3, cdw10, cdw11, cdw12, cdw13, cdw14, cdw15, data_len, data, &result);
  free(data);
  if (err < 0 || result != 0) {
    return -1;
  }

  return 0;
}
int DB::VectorBuild() {
  if (!IsOpen()) {
    return -1;
  }

  void *data = nullptr;
  unsigned int data_len = MAX_BUFLEN;
  unsigned int nlb = (MAX_BUFLEN - 1) / PAGE_SIZE;

  if (posix_memalign(&data, PAGE_SIZE, data_len)) {
    return -ENOMEM;
    std::memset(data, 0, data_len);
  }

  uint32_t result;
  uint32_t cdw2, cdw3, cdw10, cdw11, cdw12, cdw13, cdw14, cdw15;
  cdw2 = cdw3 = cdw10 = cdw11 = cdw12 = cdw13 = cdw14 = cdw15 = 0;

  cdw12 = 0 | (0xFFFF & nlb);

  int err = nvme_passthru(NVME_CMD_VECTOR_BUILD, 0, 0, NSID, cdw2, cdw3, cdw10, cdw11, cdw12, cdw13, cdw14, cdw15, data_len, data, &result);

  free(data);

  if (err < 0) {
    std::cout << "System error: " << err << strerror(-err) << std::endl;
    return -1;
  }

  if (err == 0x7C1) {
    std::cout << "Index building job is running" << std::endl;
    // index building job is running
    return -2;
  }

  if (err == 0 && result == 1) {
    std::cout << "Build started successfully" << std::endl;
    return 0;
  }
  std::cout << "Unexpected result: err=" << err << ", result=" << result << std::endl;
  return -1;
}

// may be we need to carefully consider MDTS(max data trasnfers, cosmos == 1MB)
int DB::VectorSearch(const float query_vector[VECTOR_DIMENSION], int top_k, VectorSearchReturn *rets) {
  if (!IsOpen() || !rets) {
    return -1;
  }
  void *data = nullptr;
  unsigned int data_len_nlb = 4;
  unsigned int data_len = data_len_nlb * PAGE_SIZE;
  fflush(stdout);
  if (posix_memalign(&data, PAGE_SIZE, data_len)) {
    return -ENOMEM;
  }

  std::memset(data, 0, data_len);
  std::memcpy(data, query_vector, VECTOR_SIZE);
  uint32_t result;
  uint32_t cdw2, cdw3, cdw10, cdw11, cdw12, cdw13, cdw14, cdw15;
  cdw2 = cdw3 = cdw10 = cdw11 = cdw12 = cdw13 = cdw14 = cdw15 = 0;

  // cdw12 = ((0xFFFF & data_len_nlb) - 1);
  // cdw12 = 0 | (0xFFFF & data_len_nlb);
  cdw12 = 0 | (0xFFFF & data_len_nlb);

  cdw13 = data_len;
  cdw15 = top_k;

  int err = nvme_passthru(NVME_CMD_VECTOR_SEARCH, 0, 0, NSID, cdw2, cdw3, cdw10, cdw11, cdw12, cdw13, cdw14, cdw15, data_len, data, &result);
  if (err < 0) {
    free(data);
    return -1;
  }
  if (err == 0x7C1) {
    // index not built
    free(data);
    return -2;
  }

  if (result == 0) {
    // Copy results from buffer to VectorSearchReturn
    // const char* data_ptr = static_cast<const char*>(data);
    // // Vector IDs start after the query vector
    // const unsigned int* data_ptr = reinterpret_cast<const unsigned int*>(data + (VECTOR_SIZE_NLB * PAGE_SIZE));

    const unsigned int *data_ptr = reinterpret_cast<const unsigned int *>(data);

    memcpy(rets, data_ptr, PAGE_SIZE * data_len_nlb);
  }

  free(data);
  return result;
}

int DB::PauseBuild() {
  if (!IsOpen()) {
    return -1;
  }

  void *data = nullptr;
  unsigned int data_len = 4096;

  if (posix_memalign(&data, PAGE_SIZE, data_len)) {
    return -ENOMEM;
    std::memset(data, 0, data_len);
  }

  uint32_t result;
  uint32_t cdw2, cdw3, cdw10, cdw11, cdw12, cdw13, cdw14, cdw15;
  cdw2 = cdw3 = cdw10 = cdw11 = cdw12 = cdw13 = cdw14 = cdw15 = 0;

  int err = nvme_passthru(NVME_CMD_PAUSE_BUILD, 0, 0, NSID, cdw2, cdw3, cdw10, cdw11, cdw12, cdw13, cdw14, cdw15, data_len, data, &result);

  free(data);

  std::cout << "PauseBuild: err=" << err << ", result=" << result << ", errno=" << errno << " (" << strerror(errno) << ")" << std::endl;
  if (err == 0x7C1) {
    // Pause Success
    return 1;
  } else if (err == 0) {
    // No need to pause
    return 0;
  } else {
    std::cout << "Error: err=" << err << ", result=" << result << std::endl;
    return -1;
  }
}

int DB::ResumeBuild() {
  if (!IsOpen()) {
    return -1;
  }

  void *data = nullptr;
  unsigned int data_len = 4096;

  if (posix_memalign(&data, PAGE_SIZE, data_len)) {
    return -ENOMEM;
    std::memset(data, 0, data_len);
  }

  uint32_t result;
  uint32_t cdw2, cdw3, cdw10, cdw11, cdw12, cdw13, cdw14, cdw15;
  cdw2 = cdw3 = cdw10 = cdw11 = cdw12 = cdw13 = cdw14 = cdw15 = 0;

  int err = nvme_passthru(NVME_CMD_RESUME_BUILD, 0, 0, NSID, cdw2, cdw3, cdw10, cdw11, cdw12, cdw13, cdw14, cdw15, data_len, data, &result);

  free(data);

  std::cout << "ResumeBuild: err=" << err << ", result=" << result << ", errno=" << errno << " (" << strerror(errno) << ")" << std::endl;

  if (err == 0x7C1) {
    // Resume Success
    return 1;
  } else if (err == 0) {
    // No need to resume
    return 0;
  } else {
    return -1;
  }
}

int DB::VectorBuildStatus() {
  if (!IsOpen()) {
    return -1;
  }

  void *data = nullptr;
  unsigned int data_len = 4096;

  if (posix_memalign(&data, PAGE_SIZE, data_len)) {
    return -ENOMEM;
    std::memset(data, 0, data_len);
  }

  uint32_t result;
  uint32_t cdw2, cdw3, cdw10, cdw11, cdw12, cdw13, cdw14, cdw15;
  cdw2 = cdw3 = cdw10 = cdw11 = cdw12 = cdw13 = cdw14 = cdw15 = 0;

  int err = nvme_passthru(NVME_CMD_VECTOR_BUILD_STATUS, 0, 0, NSID, cdw2, cdw3, cdw10, cdw11, cdw12, cdw13, cdw14, cdw15, data_len, data, &result);

  free(data);

  std::cout << "Check Status: err=" << err << ", result=" << result << ", errno=" << errno << " (" << strerror(errno) << ")" << std::endl;

  if (err == 0x7C1) {
    // index building job is running
    return 1;
  } else if (err == 0) {
    // Complete
    return 0;
  } else {
    return -1;
  }
}

int DB::nvme_passthru(uint8_t opcode, uint8_t flags, uint16_t rsvd, uint32_t nsid, uint32_t cdw2, uint32_t cdw3, uint32_t cdw10, uint32_t cdw11, uint32_t cdw12, uint32_t cdw13, uint32_t cdw14, uint32_t cdw15,
                      uint32_t data_len, void *data, uint32_t *result) {
  struct nvme_passthru_cmd cmd = {
      .opcode = opcode,
      .flags = flags,
      .rsvd1 = rsvd,
      .nsid = nsid,
      .cdw2 = cdw2,
      .cdw3 = cdw3,
      .metadata = (uint64_t)(uintptr_t)NULL,
      .addr = (uint64_t)(uintptr_t)data,
      .metadata_len = 0,
      .data_len = data_len,
      .cdw10 = cdw10,
      .cdw11 = cdw11,
      .cdw12 = cdw12,
      .cdw13 = cdw13,
      .cdw14 = cdw14,
      .cdw15 = cdw15,
      .timeout_ms = 0,
      .result = 0,
  };
  int err;
  err = ioctl(fd_, NVME_IOCTL_IO_CMD, &cmd);
  if (!err && result) {
    *result = cmd.result;
  }
  return err;
}

int DB::OpenMock(const std::string &dev, const std::string &collectionName) {
  std::cout << "[Mocking] Succesfully Open: " << dev << std::endl;
  return 1;
}
int DB::CloseMock() {
  std::cout << "[Mocking] Succesfully Close." << std::endl;
  return 1;
}
int DB::PutMock(const std::string &key, const float vector[VECTOR_DIMENSION], const bool delayed_compaction) { return 1; }
int DB::GetMock(const std::string &key, std::string *value) {
  std::cout << "[Mocking] Get <" << key << ">" << std::endl;
  std::string res = "MockResults";
  *value = res;
  return 1;
}
int DB::VectorSearchMock(const float query_vector[VECTOR_DIMENSION], int top_k, VectorSearchReturn *rets) {
  VectorSearchReturn res(top_k);
  unsigned int random_pivot_vid = 1234;
  float random_pivot_dist = 1.02f;

  for (unsigned int i = 0; i < VECTOR_DIMENSION; i++) {
    res.top_k_distance[i] = random_pivot_dist;
    res.top_k_vector_id[i] = random_pivot_vid;
    random_pivot_vid++;
    random_pivot_dist += 1.0f;
  }
  *rets = res;
  return 1;
}

int DB::VectorBuildMock() { return 1; }

int DB::OpenFile(const std::string &file) {
  if (fd_ != -1) {
    Close();
  }
  std::cout << "Opening the file..." << std::endl;
  int err = open(file.c_str(), O_RDWR | O_CREAT);
  if (err == -1) {
    std::cerr << "Failed to open file: " << strerror(errno) << std::endl;
    return 1;
  }
  fd_ = err;

  std::cout << "File has been connected." << fd_ << std::endl;
  return 0;
}

int DB::PutToFile(const std::string &key, const float vector[VECTOR_DIMENSION]) {
  if (!IsOpen()) {
    return -1;
  }
  std::cout << "[" << key << "] "
            << "Writing to the file..." << std::endl;
  void *data = nullptr;
  uint32_t keyLen = key.size();
  uint32_t valueLen = VECTOR_SIZE;
  size_t total_record_size = sizeof(uint32_t) + keyLen + sizeof(uint32_t) + valueLen;

  size_t alignedSize = ((total_record_size - 1) / PAGE_SIZE + 1) * PAGE_SIZE;

  if (posix_memalign(&data, PAGE_SIZE, alignedSize)) {
    return -ENOMEM;
  }
  std::memset(data, 0, alignedSize);

  char *ptr = static_cast<char *>(data);
  *reinterpret_cast<uint32_t *>(ptr) = keyLen;
  ptr += sizeof(uint32_t);

  std::memcpy(ptr, key.c_str(), keyLen);
  ptr += keyLen;

  *reinterpret_cast<uint32_t *>(ptr) = valueLen;
  ptr += sizeof(uint32_t);

  std::memcpy(ptr, vector, VECTOR_SIZE);
  ptr += VECTOR_SIZE;

  lseek(fd_, 0, SEEK_END);
  ssize_t byteWritten = write(fd_, data, VECTOR_SIZE);
  if (byteWritten == -1) {
    std::cerr << "Failed to write: " << strerror(errno) << std::endl;
    close(fd_);
    return 1;
  }
  free(data);

  std::cout << "Put has been succesffuly performed to the file." << std::endl;

  return 0;
}

int DB::GetFromFile(const std::string &key, std::string *value) {
  if (!IsOpen()) {
    return -1;
  }
  std::cout << "Reading from the file..." << std::endl;
  void *data = nullptr;
  unsigned int data_len = MAX_BUFLEN;
  unsigned int nlb = (MAX_BUFLEN - 1) / PAGE_SIZE;
  if (posix_memalign(&data, PAGE_SIZE, data_len)) {
    return -ENOMEM;
  }
  std::memset(data, 0, data_len);
  uint32_t result;

  lseek(fd_, 0, SEEK_SET);
  ssize_t byteRead = read(fd_, data, data_len);
  if (byteRead == -1) {
    std::cerr << "Failed to read: " << strerror(errno) << std::endl;
    close(fd_);
    free(data);
    return -1;
  }
  float *fdata = reinterpret_cast<float *>(data);
  size_t num_floats = byteRead / sizeof(float);
  std::cout << "Read float32 values (" << num_floats << ") :" << std::endl;
  for (size_t i = 0; i < num_floats; ++i) {
    std::cout << "  [" << i << "]: " << fdata[i];
  }
  std::cout << std::endl;

  if (value) {
    *value = std::string(static_cast<const char *>(data), byteRead);
  }

  free(data);
  std::cout << "Get has been succesffuly performed from the file." << std::endl;
  return byteRead;
}
} // namespace vectorssd

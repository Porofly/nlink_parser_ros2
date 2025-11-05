#ifndef PROTOCOL_MANAGER_H
#define PROTOCOL_MANAGER_H

#include <memory>
#include <vector>
#include "protocol_extracter/nprotocol_base.h"
#include "protocol_extracter/nprotocol_extracter.h"

class ProtocolManager {
public:
  ProtocolManager() = default;
  ~ProtocolManager() = default;

  ProtocolManager(const ProtocolManager&) = delete;
  ProtocolManager& operator=(const ProtocolManager&) = delete;
  ProtocolManager(ProtocolManager&&) = default;
  ProtocolManager& operator=(ProtocolManager&&) = default;

  template<typename ProtocolType>
  ProtocolType* addProtocol(NProtocolExtracter* extracter) {
    auto protocol = std::make_unique<ProtocolType>();
    ProtocolType* raw_ptr = protocol.get();

    extracter->AddProtocol(raw_ptr);
    protocols_.push_back(std::move(protocol));

    return raw_ptr;
  }

private:
  std::vector<std::unique_ptr<NProtocolBase>> protocols_;
};

#endif

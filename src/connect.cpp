#include "connect.hpp"

namespace Connect {

void TransferProtocol::write( std::span< const DataType > data ) const {}

std::vector< TransferProtocol::DataType > TransferProtocol::read() const { return {}; }

void TransferProtocol::begin() {}

void TransferProtocol::end() {}
}   // namespace Connect

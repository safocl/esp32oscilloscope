#pragma once

#include "utils.hpp"

#include "asio/buffer.hpp"
#include "asio/error_code.hpp"
#include "asio/ip/basic_endpoint.hpp"
#include "esp_netif_ip_addr.h"
#include <asio/io_context.hpp>
#include <asio/ip/tcp.hpp>
#include <asio.hpp>

#include <algorithm>
#include <bit>
#include <charconv>
#include <concepts>
#include <cstddef>
#include <format>
#include <memory>
#include <print>
#include <ranges>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>
#include <cstdint>
#include <span>

namespace Connect {
class IpV4 final {
    /*
	* "127. 0 . 0 . 1"
	*  [0],[1],[2],[3]
	*
	*/
public:
    using OctetType     = std::uint8_t;
    using ArrayOfOctets = std::array< OctetType, 4 >;

    constexpr IpV4( ArrayOfOctets octets ) : mOctets( octets ) {}

    constexpr IpV4( esp_ip4_addr_t addr ) : mOctets( toBytes< std::uint8_t >( addr.addr ) ) {}

    constexpr IpV4( std::string_view ipStr ) { from( ipStr ); }

    constexpr void from( ArrayOfOctets octets ) { mOctets = octets; }
    constexpr void from( esp_ip4_addr_t addr ) { mOctets = toBytes< std::uint8_t >( addr.addr ); }
    constexpr void from( std::string_view ipStr ) {
        static_assert( 1 == sizeof( ArrayOfOctets::value_type ) );

        ArrayOfOctets a;

        for ( auto & el : a ) {
            auto [ ptr, ec ] = std::from_chars( ipStr.data(), ipStr.data() + ipStr.size(), el );
            if ( ec != std::errc() )
                throw std::runtime_error( "IP string is invalid." );
            ipStr.remove_prefix(
            std::ranges::min( std::distance( ipStr.data(), ptr ) + 1, std::ranges::ssize( ipStr ) ) );
        }

        mOctets = a;
    }

    constexpr ArrayOfOctets asOctets() const { return mOctets; }
    std::string             asString() const {
        return std::format( "{}.{}.{}.{}", mOctets[ 0 ], mOctets[ 1 ], mOctets[ 2 ], mOctets[ 3 ] );
    }

    constexpr operator esp_ip4_addr_t() const { return { fromBytes( mOctets ) }; }

    constexpr OctetType operator[]( ArrayOfOctets::size_type pos ) const { return mOctets[ pos ]; }
    constexpr IpV4 &    operator&=( const IpV4 & mask ) {
        for ( auto i : std::views::iota( 0, 4 ) )
            mOctets[ i ] &= mask.mOctets[ i ];
        return *this;
    }

private:
    ArrayOfOctets mOctets;
};

constexpr IpV4 operator&( const IpV4 & lhs, const IpV4 & rhs ) {
    IpV4 ret( lhs );
    ret &= rhs;
    return ret;
}

inline std::string to_string( IpV4 ip ) { return ip.asString(); }

constexpr inline IpV4::ArrayOfOctets to_array( IpV4 ip ) { return ip.asOctets(); }

enum class Port : std::uint_least16_t {};

constexpr inline std::string to_string( Port port ) { return std::to_string( std::to_underlying( port ) ); }

class TransferProtocol final : public std::enable_shared_from_this< TransferProtocol > {
    struct CreateInfo {
        const asio::ip::tcp::endpoint remote;
        const asio::ip::tcp::endpoint local;
    };

public:
    using DataType = std::byte;

    explicit TransferProtocol( CreateInfo ci ) {
#if 1
        std::println( "-----Bind to: {}:{}", ci.local.address().to_v4().to_string(), ci.local.port() );
#endif

        mRemote.connect( ci.remote );

        mLocal.open( ci.local.protocol() );
        mLocal.bind( ci.local );
    }

    static std::shared_ptr< TransferProtocol > create( asio::ip::tcp::endpoint remote, asio::ip::tcp::endpoint local ) {
        return std::make_shared< TransferProtocol >( CreateInfo { remote, local } );
    }

    static std::shared_ptr< TransferProtocol >
    create( IpV4 remoteAddr, Port remotePort, IpV4 localAddr, Port localPort ) {
        return std::make_shared< TransferProtocol >(
        CreateInfo { { asio::ip::make_address_v4( remoteAddr.asOctets() ), std::to_underlying( remotePort ) },
                     { asio::ip::make_address_v4( localAddr.asOctets() ), std::to_underlying( localPort ) } } );
    }

    ~TransferProtocol() {
        // mRemote.cancel();
        // mRemote.close();
        //
        // mLocal.cancel();
        // mLocal.close();
        //
        // mClientCtx.stop();
        // mServerCtx.stop();
    }

    void stop() {
        mRemote.cancel();
        mLocal.cancel();
    }

    void write( std::span< const DataType > data, std::invocable< asio::error_code, std::size_t > auto compliteCb );
    void write( std::span< const DataType > data );

    std::vector< DataType > read( std::size_t maxSize );

    bool hasConnection() const { return mRemote.is_open(); }

    void                            waitForDone() { mClientCtx.run(); }
    template < class... Args > void waitForDone( std::invocable< Args... > auto && cb, Args &&... args ) {
        mClientCtx.run();
        std::invoke( cb, args... );
    }

private:
    asio::io_context mServerCtx;
    asio::io_context mClientCtx;

    asio::ip::tcp::socket mRemote { mClientCtx };
    asio::ip::tcp::socket mLocal { mServerCtx };
};

inline void TransferProtocol::write( std::span< const DataType >                          data,
                                     std::invocable< asio::error_code, std::size_t > auto compliteCb ) {
#if 0
    std::print( "-----Transmit start {} bytes to: {}:{}\n",
                data.size_bytes(),
                mDestination.remote_endpoint().address().to_v4().to_string(),
                mDestination.remote_endpoint().port() );
#endif
    asio::async_write( mRemote, asio::buffer( data ), compliteCb );
}

inline void TransferProtocol::write( std::span< const DataType > data ) {
#if 0
    std::print( "-----Transmit start {} bytes to: {}:{}\n",
                data.size_bytes(),
                mDestination.remote_endpoint().address().to_v4().to_string(),
                mDestination.remote_endpoint().port() );
#endif
    asio::write( mRemote, asio::buffer( data ) );
}

inline std::vector< TransferProtocol::DataType > TransferProtocol::read( std::size_t maxSize ) { return {}; }
}   // namespace Connect

#pragma once

#include <algorithm>
#include <bit>
#include <concepts>
#include <cstdint>
#include <cstddef>
#include <array>
#include <initializer_list>
#include <span>

template < class... Ts > [[nodiscard]] consteval auto maxSizeOfTypes() noexcept {
    return std::ranges::max( std::initializer_list< std::size_t > {
    sizeof( Ts )...,
    } );
}

constexpr auto asLeastNearestMultiple( std::integral auto lowestThreshold, std::integral auto n )
-> std::common_type_t< decltype( lowestThreshold ), decltype( n ) > {
    return ( ( lowestThreshold + n - 1 ) / n ) * n;
}

template < std::endian From, std::endian To > constexpr auto swapEndian( std::integral auto i ) {
    if constexpr ( ( std::endian::native != std::endian::little ) && ( std::endian::native != std::endian::big ) )
        static_assert( false, "Mixed Endian is unsupported" );

    if constexpr ( From != To )
        i = std::byteswap( i );

    return i;
}

template < class T >
concept ByteTypeConcept = std::same_as< T, std::uint8_t > || std::same_as< T, std::byte >;

template < ByteTypeConcept ByteType > constexpr auto toBytes( std::integral auto i ) {
    return std::bit_cast< std::array< ByteType, sizeof( i ) > >( i );
}

template < ByteTypeConcept ByteType > constexpr auto toBigEndianBytes( std::integral auto i ) {
    return toBytes< ByteType >( swapEndian< std::endian::native, std::endian::big >( i ) );
}

constexpr auto fromBigEndian( std::integral auto i ) {
    return swapEndian< std::endian::big, std::endian::native >( i );
}

template < ByteTypeConcept ByteType, std::size_t N >
    requires( N == 2 || N == 4 || N == 8 )
inline auto fromBytes( std::array< ByteType, N > bytes ) {
    if constexpr ( N == 8 )
        return std::bit_cast< std::uint64_t >( bytes );
    else if constexpr ( N == 4 )
        return std::bit_cast< std::uint32_t >( bytes );
    else if constexpr ( N == 2 )
        return std::bit_cast< std::uint16_t >( bytes );
}

template < ByteTypeConcept ByteType, std::size_t N > inline auto fromBigEndianBytes( std::array< ByteType, N > bytes ) {
    return fromBigEndian( fromBytes( bytes ) );
}

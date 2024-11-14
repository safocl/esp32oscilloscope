#pragma once

#include <esp_netif_types.h>
#include <esp_netif.h>
#include <esp_exception.hpp>
#include <concepts>
#include <span>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string_view>

namespace core {

class NetIf;
class NetIfHandler;

class DhcpcOption final {
    explicit DhcpcOption( esp_netif_t * ptr ) noexcept : mNetIf( ptr ) {}

public:
    friend class NetIfHandler;

    void setSubnetMask() { mNetIf = nullptr; }
    void setDomainNameServer( std::string_view mask ) {}
    void setRouterSolicitationAddress( std::string_view mask ) {}
    void setRequestedIpAddress( std::string_view mask ) {}
    void setIpAddressLeaseTime( esp_netif_dhcp_option_id_t ) {}
    void setIpRequestRetryTime() {}
    void setVendorClassIdentifier() {}
    void setVendorSpecificInfo() {}

    void getSubnetMask() {}
    void getDomainNameServer() {}
    void getRouterSolicitationAddress() {}
    void getRequestedIpAddress() {}
    void getIpAddressLeaseTime() {}
    void getIpRequestRetryTime() {}
    void getVendorClassIdentifier() {}
    void getVendorSpecificInfo() {}

private:
    esp_netif_t * mNetIf {};
};

class DhcpsOption final {
    explicit DhcpsOption( esp_netif_t * ptr ) noexcept : mNetIf( ptr ) {}

public:
    friend class NetIfHandler;

    void setSubnetMask() { mNetIf = nullptr; }
    void setDomainNameServer( std::string_view mask ) {}
    void setRouterSolicitationAddress( std::string_view mask ) {}
    void setRequestedIpAddress( std::string_view mask ) {}
    void setIpAddressLeaseTime( esp_netif_dhcp_option_id_t ) {}
    void setIpRequestRetryTime() {}
    void setVendorClassIdentifier() {}
    void setVendorSpecificInfo() {}

    void getSubnetMask() {}
    void getDomainNameServer() {}
    void getRouterSolicitationAddress() {}
    void getRequestedIpAddress() {}
    void getIpAddressLeaseTime() {}
    void getIpRequestRetryTime() {}
    void getVendorClassIdentifier() {}
    void getVendorSpecificInfo() {}

private:
    esp_netif_t * mNetIf {};
};

class NetIfHandler final {
    struct NetIfDeleter final {
        void operator()( esp_netif_t * ptr ) { esp_netif_destroy( ptr ); }
    };

    operator esp_netif_t *() { return mHandler.get(); }

    template < std::invocable< esp_netif_t * > Deleter >
    NetIfHandler( esp_netif_t * ptr, Deleter && deleter ) : mHandler( ptr, deleter ) {
        if ( !static_cast< bool >( mHandler ) )
            throw std::runtime_error( "NetIfHandler don't created!!!" );
    }

    explicit NetIfHandler( const esp_netif_config_t & esp_netif_config ) :
    mHandler( esp_netif_new( &esp_netif_config ), NetIfDeleter() ) {
        if ( !static_cast< bool >( mHandler ) )
            throw std::runtime_error( "NetIfHandler don't created!!!" );
    }

public:
    friend class NetIf;

    NetIfHandler() noexcept = default;

    NetIfHandler( NetIfHandler && ) noexcept = default;
    NetIfHandler( const NetIfHandler & )     = delete;

    NetIfHandler & operator=( NetIfHandler && ) noexcept = default;
    NetIfHandler & operator=( const NetIfHandler & )     = delete;

    ~NetIfHandler() noexcept = default;

    operator bool() noexcept { return static_cast< bool >( mHandler ); }

    void setDriverConfig( const esp_netif_driver_ifconfig_t * driver_config ) {
        esp_netif_set_driver_config( *this, driver_config );
    }

    void attach( esp_netif_iodriver_handle driver_handle ) { esp_netif_attach( *this, driver_handle ); }

    void receive( void * buffer, size_t len, void * eb ) { esp_netif_receive( *this, buffer, len, eb ); }

    void setDefaultNetif() { esp_netif_set_default_netif( *this ); }

    void joinIp6Multicast_group( const esp_ip6_addr_t * addr ) { esp_netif_join_ip6_multicast_group( *this, addr ); }

    void leaveIp6MulticastGroup( const esp_ip6_addr_t * addr ) { esp_netif_leave_ip6_multicast_group( *this, addr ); }

    void setMac( uint8_t mac[] ) { esp_netif_set_mac( *this, mac ); }

    void getMac( uint8_t mac[] ) { esp_netif_get_mac( *this, mac ); }

    void setHostname( std::string_view hostname ) { esp_netif_set_hostname( *this, hostname.data() ); }

    std::string getHostname() {
        const char * hostname {};
        CHECK_THROW( esp_netif_get_hostname( *this, &hostname ) );

        return { hostname };
    }

    bool isNetifUp() { return esp_netif_is_netif_up( *this ); }

    esp_netif_ip_info_t getIpInfo() {
        esp_netif_ip_info_t ip_info {};
        esp_netif_get_ip_info( *this, &ip_info );
        return ip_info;
    }

    esp_netif_ip_info_t getOldIpInfo() {
        esp_netif_ip_info_t ip_info {};
        esp_netif_get_old_ip_info( *this, &ip_info );
        return ip_info;
    }

    void setIpInfo( const esp_netif_ip_info_t & ip_info ) { CHECK_THROW( esp_netif_set_ip_info( *this, &ip_info ) ); }

    void setOldIpInfo( const esp_netif_ip_info_t * ip_info ) { esp_netif_set_old_ip_info( *this, ip_info ); }

    int getNetifImplIndex() { return esp_netif_get_netif_impl_index( *this ); }

    void getNetifImplName( char * name ) { esp_netif_get_netif_impl_name( *this, name ); }

    void naptEnable() { esp_netif_napt_enable( *this ); }

    void naptDisable() { esp_netif_napt_disable( *this ); }

    DhcpsOption dhcpsOption() { return DhcpsOption( static_cast< esp_netif_t * >( *this ) ); }

    DhcpcOption dhcpcOption() { return DhcpcOption( static_cast< esp_netif_t * >( *this ) ); }

    void dhcpcStart() { esp_netif_dhcpc_start( *this ); }

    void dhcpcStop() { esp_netif_dhcpc_stop( *this ); }

    esp_netif_dhcp_status_t dhcpcGetStatus() {
        esp_netif_dhcp_status_t status {};
        esp_netif_dhcpc_get_status( *this, &status );
        return status;
    }

    esp_netif_dhcp_status_t dhcpsGetStatus() {
        esp_netif_dhcp_status_t status {};
        esp_netif_dhcps_get_status( *this, &status );
        return status;
    }

    void dhcpsStart() { CHECK_THROW( esp_netif_dhcps_start( *this ) ); }

    void dhcpsStop() { esp_netif_dhcps_stop( *this ); }

    void dhcpsGetClientsByMac( std::span< esp_netif_pair_mac_ip_t > macIpPairs ) {
        CHECK_THROW( esp_netif_dhcps_get_clients_by_mac( *this, macIpPairs.size(), macIpPairs.data() ) );
    }

    void setDnsInfo( esp_netif_dns_type_t type, esp_netif_dns_info_t * dns ) {
        esp_netif_set_dns_info( *this, type, dns );
    }

    void getDnsInfo( esp_netif_dns_type_t type, esp_netif_dns_info_t * dns ) {
        esp_netif_get_dns_info( *this, type, dns );
    }

#if CONFIG_LWIP_IPV6

    void createIp6Linklocal() { esp_netif_create_ip6_linklocal( *this ); }

    void getIp6Linklocal( esp_ip6_addr_t * if_ip6 ) { esp_netif_get_ip6_linklocal( *this, if_ip6 ); }

    void getIp6Global( esp_ip6_addr_t * if_ip6 ) { esp_netif_get_ip6_global( *this, if_ip6 ); }

    int getAllIp6( esp_ip6_addr_t if_ip6[] ) { return esp_netif_get_all_ip6( *this, if_ip6 ); }
#endif

#if CONFIG_ESP_NETIF_BRIDGE_EN

    void bridgeAddPort( _br, esp_netif_t * esp_netif_port ) {
        esp_netif_bridge_add_port( esp_netif_br, esp_netif_port );
    }

    void bridgeFdbAdd( _br, uint8_t * addr, uint64_t ports_mask ) {
        esp_netif_bridge_fdb_add( esp_netif_br, addr, ports_mask );
    }

    void bridgeFdbRemove( _br, uint8_t * addr ) { esp_netif_bridge_fdb_remove( esp_netif_br, addr ); }
#endif   // CONFIG_ESP_NETIF_BRIDGE_EN

    esp_netif_iodriver_handle getIoDriver() { return esp_netif_get_io_driver( *this ); }

    esp_netif_flags_t getFlags() { return esp_netif_get_flags( *this ); }

    std::string_view espNetifGetIfkey() { return esp_netif_get_ifkey( *this ); }

    std::string_view espNetifGetDesc() { return esp_netif_get_desc( *this ); }

    int getRoutePrio() { return esp_netif_get_route_prio( *this ); }

    int32_t getEventId( esp_netif_ip_event_type_t event_type ) { return esp_netif_get_event_id( *this, event_type ); }

    esp_netif_t * espNetifNextUnsafe() { return esp_netif_next_unsafe( *this ); }

private:
    std::unique_ptr< esp_netif_t, std::function< void( esp_netif_t * ) > > mHandler { nullptr };
};

class NetIf final {
public:
    static void init() {
        if ( isInited )
            return;

        CHECK_THROW( esp_netif_init() );

        isInited = true;
    }

    static void deinit() noexcept {}

    template < std::invocable< esp_netif_t * > Deleter >
    static NetIfHandler createHandler( esp_netif_t * ptr, Deleter && deleter ) {
        return NetIfHandler { ptr, deleter };
    }

    static NetIfHandler createHandler( const esp_netif_config_t & esp_netif_config ) {
        return NetIfHandler { esp_netif_config };
    }

    static NetIfHandler getDefaultNetif() {
        return NetIfHandler( esp_netif_get_default_netif(), []( esp_netif_t * ) {} );
    }

private:
    inline static bool isInited = false;
};

}   // namespace core

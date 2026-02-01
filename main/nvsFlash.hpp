#pragma once

#include <nvs_flash.h>
#include <esp_exception.hpp>

namespace core {

class NvsFlash final {
public:
    static void init() {
        if ( isInited )
            return;

        esp_err_t ret = nvs_flash_init();
        if ( ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND ) {
            ESP_ERROR_CHECK( nvs_flash_erase() );
            ret = nvs_flash_init();
        }
        CHECK_THROW( ret );
    }

private:
    inline static bool isInited = false;
};

}   // namespace core

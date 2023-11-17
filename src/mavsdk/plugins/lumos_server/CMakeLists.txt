target_sources(mavsdk
    PRIVATE
    lumos_server.cpp
    lumos_server_impl.cpp
	dance_data.cpp
)

target_include_directories(mavsdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/mavsdk>
    )

install(FILES
    include/plugins/lumos_server/lumos_server.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mavsdk/plugins/lumos_server
)

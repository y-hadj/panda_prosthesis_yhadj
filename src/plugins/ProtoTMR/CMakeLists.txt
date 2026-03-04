add_library(ProtoTMR SHARED ProtoTMRSerial.cpp)
target_link_libraries(ProtoTMR PUBLIC mc_rtc::mc_rtc_utils)

install(TARGETS ProtoTMR
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)


set(AUTOLOAD_ProtoTMRPlugin_PLUGIN
    OFF
    CACHE INTERNAL "Automatically load ProtoTMR plugin"
)


set(plugin_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/ProtoTMRPlugin.cpp
)

set(plugin_HDR
  ${CMAKE_CURRENT_SOURCE_DIR}/ProtoTMRPlugin.h
)
add_plugin(ProtoTMRPlugin "${plugin_SRC}" "${plugin_HDR}")
# target_link_libraries(BoneTagSerialPlugin PUBLIC BoneTagSerial)
target_link_libraries(ProtoTMRPlugin PUBLIC ProtoTMR)
install(FILES etc/ProtoTMRPlugin.yaml DESTINATION ${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc/)

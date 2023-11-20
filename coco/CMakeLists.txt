add_library(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
	PUBLIC FILE_SET headers TYPE HEADERS FILES
		ieee802154.hpp
		Ieee802154Radio.hpp
	PRIVATE
		Ieee802154Radio.cpp
)

if(${PLATFORM} STREQUAL "native" OR ${PLATFORM} STREQUAL "emu")
	# native platform (Windows, MacOS, Linux)
	target_sources(${PROJECT_NAME}
		PUBLIC FILE_SET platform_headers TYPE HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/native FILES
			native/coco/platform/Ieee802154Radio_usb.hpp
		PRIVATE
			native/coco/platform/Ieee802154Radio_usb.cpp
	)
elseif(${PLATFORM} MATCHES "^nrf52")
	target_sources(${PROJECT_NAME}
		PUBLIC FILE_SET platform_headers TYPE HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/nrf52 FILES
			nrf52/coco/platform/Ieee802154Radio_RADIO_TIMER0.hpp
		PRIVATE
			nrf52/coco/platform/Ieee802154Radio_RADIO_TIMER0.cpp
	)
endif()

target_link_libraries(${PROJECT_NAME}
	coco::coco
	coco-loop::coco-loop
	coco-buffer::coco-buffer
)

# link to coco-usb on native platforms
if(${PLATFORM} STREQUAL "native" OR ${PLATFORM} STREQUAL "emu")
	target_link_libraries(${PROJECT_NAME}
		coco-usb::coco-usb
	)
endif()

target_include_directories(${PROJECT_NAME}
	PUBLIC
		..
)

# install the library
install(TARGETS ${PROJECT_NAME}
	FILE_SET headers DESTINATION include/coco
	FILE_SET platform_headers DESTINATION include
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

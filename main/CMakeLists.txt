
set(sources 
	Agent.cpp
	Episode.cpp
	Simulator.cpp
	StateActionTable.cpp
	main.cpp
	)

idf_component_register(SRCS ${sources}
                    INCLUDE_DIRS ".")
                    
idf_build_set_property(COMPONENTS driver/i2c bootloader_support)



add_library(TeensyStepC STATIC)

target_include_directories(TeensyStepC PUBLIC src)

target_sources(TeensyStepC PRIVATE 
    src/ErrorHandler.c
    src/MotorControlBase.c
    src/RotateControlBase.c
    src/StepControlBase.c
    src/Stepper.c
)

target_sources(TeensyStepC PRIVATE 
    src/accelerators/LinRotAccelerator.c
    src/accelerators/LinStepAccelerator.c
    src/accelerators/SinRotAccelerator.c
)


target_sources(TeensyStepC PRIVATE 
    src/port/port.h
)

target_compile_definitions(TeensyStepC PRIVATE -DNDEBUG)
target_compile_options(TeensyStepC PRIVATE -ffunction-sections -fdata-sections)
target_compile_options(TeensyStepC PRIVATE -Os)

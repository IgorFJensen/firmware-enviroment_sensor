idf_component_register(
    SRCS "tinyml_runtime.cpp"
    INCLUDE_DIRS "."
    REQUIRES tinyml_model esp-tflite-micro
)
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)

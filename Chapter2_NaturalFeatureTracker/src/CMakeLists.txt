add_library(nft
    CameraCalibration.cpp
    CameraCalibration.hpp
    GeometryTypes.cpp
    GeometryTypes.hpp
    Pattern.cpp
    Pattern.hpp
    PatternDetector.cpp
    PatternDetector.hpp
    DebugHelpers.hpp
    ARPipeline.hpp
    ARPipeline.cpp
)

add_executable(nft_demo 
    ARDrawingContext.cpp
    ARDrawingContext.hpp
    main.cpp
)

target_link_libraries(nft ${OpenCV_LIBRARIES})

target_link_libraries(nft_demo nft)
target_link_libraries(nft_demo ${OpenCV_LIBRARIES})
target_link_libraries(nft_demo ${OPENGL_LIBRARIES})
install (TARGETS nft_demo DESTINATION bin)
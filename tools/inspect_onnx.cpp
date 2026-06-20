#include <iostream>
#include <onnxruntime_cxx_api.h>

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "usage: inspect_onnx model.onnx\n";
        return 2;
    }

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "inspect");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    Ort::Session session(env, argv[1], options);
    Ort::AllocatorWithDefaultOptions allocator;

    std::cout << "MODEL " << argv[1] << "\n";
    std::cout << "inputs " << session.GetInputCount() << "\n";
    for (size_t i = 0; i < session.GetInputCount(); ++i)
    {
        auto name = session.GetInputNameAllocated(i, allocator);
        auto type_info = session.GetInputTypeInfo(i);
        auto info = type_info.GetTensorTypeAndShapeInfo();
        std::cout << " input[" << i << "] " << name.get()
                  << " type=" << info.GetElementType() << " shape=";
        for (auto dim : info.GetShape())
            std::cout << dim << ",";
        std::cout << "\n";
    }

    std::cout << "outputs " << session.GetOutputCount() << "\n";
    for (size_t i = 0; i < session.GetOutputCount(); ++i)
    {
        auto name = session.GetOutputNameAllocated(i, allocator);
        auto type_info = session.GetOutputTypeInfo(i);
        auto info = type_info.GetTensorTypeAndShapeInfo();
        std::cout << " output[" << i << "] " << name.get()
                  << " type=" << info.GetElementType() << " shape=";
        for (auto dim : info.GetShape())
            std::cout << dim << ",";
        std::cout << "\n";
    }

    return 0;
}

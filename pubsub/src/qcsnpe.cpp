/**
 * @file qcsnpe.cpp
 *
 * @brief Implementation of Qcsnpe Class. 
 *
 * @author Arunraj A P
 *
 */

#include <qcsnpe.hpp>

/************************************************************************
* Name : Qcsnpe <Constructor>
* Function: Constructor checks runtime availability, set runtime, set
*           output layers and then loads the DLC model to model handler 
************************************************************************/
Qcsnpe::Qcsnpe(std::string &dlc, std::vector<std::string> &output_layers, int system_type) {
    
    std::ifstream dlc_file(dlc);
    zdl::DlSystem::Runtime_t runtime_cpu = zdl::DlSystem::Runtime_t::CPU;
    zdl::DlSystem::Runtime_t runtime_gpu = zdl::DlSystem::Runtime_t::GPU;
    zdl::DlSystem::Runtime_t runtime_dsp = zdl::DlSystem::Runtime_t::DSP;
    zdl::DlSystem::Runtime_t runtime_aip = zdl::DlSystem::Runtime_t::AIP_FIXED8_TF;

    if (!dlc_file) {
        std::cerr << "Dlc file not valid. Please ensure that you have provided a valid dlc for processing." << std::endl;
        exit(0);
    }

    //Loading Model and setting Runtime
    static zdl::DlSystem::Version_t Version = zdl::SNPE::SNPEFactory::getLibraryVersion();
    std::cout << "SNPE Version: " << Version.asString().c_str() << std::endl; //Print Version number
    container = zdl::DlContainer::IDlContainer::open(zdl::DlSystem::String(dlc.c_str()));
    if (container == nullptr) {
        std::cerr << "Error while opening the container file." << std::endl;
        exit(0);
    }

    switch(system_type) {
        case 3: if(zdl::SNPE::SNPEFactory::isRuntimeAvailable(runtime_aip)) {
                    runtime_list.add(runtime_aip);
                    std::cout<<"AIP added to runtime list"<<std::endl;
                }   
                else 
                    std::cerr<<"AIP not available"<<std::endl;
        case 2: if(zdl::SNPE::SNPEFactory::isRuntimeAvailable(runtime_dsp)) {
                    runtime_list.add(runtime_dsp);
                    std::cout<<"DSP added to runtime list"<<std::endl;
                }
                else 
                    std::cerr<<"DSP not available"<<std::endl;
        case 1: if(zdl::SNPE::SNPEFactory::isRuntimeAvailable(runtime_gpu)) {
                    runtime_list.add(runtime_gpu);
                    std::cout<<"GPU added to runtime list"<<std::endl;
                }
                else 
                    std::cerr<<"GPU not available"<<std::endl;
        case 0: if(zdl::SNPE::SNPEFactory::isRuntimeAvailable(runtime_cpu)) {
                    runtime_list.add(runtime_cpu);
                    std::cout<<"CPU added to runtime list"<<std::endl;
                }
                else 
                    std::cerr<<"CPU not available"<<std::endl;
                break;
        default: std::cerr<<"Runtime invalid. Setting to CPU"<<std::endl; 
                 runtime_list.add(runtime_cpu);
                 break;
    }
    if(runtime_list.size()>1) {
        std::cout<<"Multiple runtime available. Fallback enabled"<<std::endl;
    }
    zdl::SNPE::SNPEBuilder snpeBuilder(container.get());
    std::cout<<"Done snpeBuilder 1"<<std::endl;
    for(auto &output_layer: output_layers) {
        outputs.append(output_layer.c_str());
    }
    std::cout<<"Done snpeBuilder 2"<<std::endl;
    model_handler = snpeBuilder.setOutputLayers(outputs)
        .setRuntimeProcessorOrder(runtime_list)
        .build();
    std::cout<<"Done snpeBuilder 3"<<std::endl;
    if (model_handler == nullptr) {
        std::cerr << "Error during creation of SNPE object." << std::endl;
        exit(0);
    }
    std::cout<<"Done snpeBuilder 4"<<std::endl;
}

Qcsnpe::Qcsnpe(const Qcsnpe& qc) {
    model_handler = std::move(qc.model_handler);
    container = std::move(qc.container);
    runtime_list = qc.runtime_list;
    outputs = qc.outputs;
    output_tensor_map = qc.output_tensor_map;
    out_tensors = qc.out_tensors;
}

/************************************************************************
* Name : predict
* Function: Method of qcsnpe class for infrencing
* Returns: A STL map with output tensor as key and its corresponding
*          output as value of map.
************************************************************************/
std::map<std::string, std::vector<float>> Qcsnpe::predict(cv::Mat input_image) {
    unsigned long int in_size = 1;
    const zdl::DlSystem::TensorShape i_tensor_shape = model_handler->getInputDimensions();
    const zdl::DlSystem::Dimension *shapes = i_tensor_shape.getDimensions();
    size_t img_size = input_image.channels() * input_image.cols * input_image.rows;
    for(int i=1; i<i_tensor_shape.rank(); i++) {
      in_size *= shapes[i];
    }

    if(in_size != img_size) {
        std::cout<<"Input Size mismatch!"<<std::endl;
        std::cout<<"Expected: "<<in_size<<std::endl;
        std::cout<<"Got: "<<img_size<<std::endl;
        exit(0);
    }
    std::unique_ptr<zdl::DlSystem::ITensor> input_tensor =
        zdl::SNPE::SNPEFactory::getTensorFactory().createTensor(model_handler->getInputDimensions());
    zdl::DlSystem::ITensor *tensor_ptr = input_tensor.get();
    if(tensor_ptr == nullptr) {
        std::cerr << "Could not create SNPE input tensor" << std::endl;
        exit(0);
    }

    //Preprocess
    float *tensor_ptr_fl = reinterpret_cast<float *>(&(*input_tensor->begin()));
    for(auto i=0; i<img_size; i++) {
        tensor_ptr_fl[i] = (static_cast<float>(input_image.data[i]) - 128.f )/ 128.f;
    }

    //infer
    auto start = std::chrono::high_resolution_clock::now();
    bool exec_status = model_handler->execute(tensor_ptr, output_tensor_map);
    auto end = std::chrono::high_resolution_clock::now();
    if (!exec_status) {
        std::cerr << "Error while executing the network." << std::endl;
        exit(0);
    }
    std::chrono::duration<float> elapsed_time = (end - start);
    std::chrono::milliseconds d = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time);
    std::cout<<"Time taken: "<<d.count()<<" ms"<<std::endl;
    std::cout<<"FPS: "<<1.0/elapsed_time.count()<<std::endl;
    throughput_vec.push_back(d.count());
    fps_vec.push_back(1.0/elapsed_time.count());

    //postprocess
    out_tensors = output_tensor_map.getTensorNames();
    std::map<std::string, std::vector<float>> out_itensor_map;
    for(size_t i=0; i<out_tensors.size(); i++) {
        zdl::DlSystem::ITensor *out_itensor = output_tensor_map.getTensor(out_tensors.at(i));
        std::vector<float> out_vec { reinterpret_cast<float *>(&(*out_itensor->begin())), reinterpret_cast<float *>(&(*out_itensor->end()))};
        out_itensor_map.insert(std::make_pair(std::string(out_tensors.at(i)), out_vec));
    }
    
    return out_itensor_map;
}

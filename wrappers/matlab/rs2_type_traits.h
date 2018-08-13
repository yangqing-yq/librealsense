#pragma once
#include "librealsense2/rs.hpp"
#include <type_traits>

// extra checks to help specialization priority resoultion
template <typename T> using extra_checks = std::bool_constant<std::is_base_of<rs2::frame, T>::value
//                                                           || std::is_base_of<rs2::options, T>::value
                                                           || std::is_base_of<rs2::stream_profile, T>::value>;

// C API

// rs_types.hpp

// rs_frame.hpp
template<typename T> struct MatlabParamParser::type_traits<T, typename std::enable_if<std::is_base_of<rs2::stream_profile, T>::value>::type> {
    using rs2_internal_t = const rs2_stream_profile*;
    using use_cells = std::true_type;
    static T from_internal(rs2_internal_t * ptr) { return T(rs2::stream_profile(*ptr)); }
};
template<typename T> struct MatlabParamParser::type_traits<T, typename std::enable_if<std::is_base_of<rs2::frame, T>::value>::type> { using rs2_internal_t = rs2_frame * ; };

// rs_sensor.hpp
template<> struct MatlabParamParser::type_traits<rs2::options> {
    struct carrier {
        void * ptr;
        enum class types { rs2_sensor, rs2_process_interface, rs2_colorizer, rs2_pointcloud } type;
        carrier(void *ptr_, types t) : ptr(ptr_), type(t) {}
        ~carrier(); // implemented at the bottom with an explanation as to why
    };
    using rs2_internal_t = carrier;
    static rs2::options from_internal(rs2_internal_t * ptr);
};
template<typename T> struct MatlabParamParser::type_traits<T, typename std::enable_if<std::is_base_of<rs2::sensor, T>::value>::type>
    : MatlabParamParser::type_traits<rs2::options> {
    using carrier_t = std::shared_ptr<rs2_sensor>;
    using carrier_enum = std::integral_constant<rs2_internal_t::types, rs2_internal_t::types::rs2_sensor>;
    static T from_internal(rs2_internal_t * ptr) {
        if (ptr->type == carrier_enum::value) return T(rs2::sensor(*reinterpret_cast<carrier_t*>(ptr->ptr)));
        mexErrMsgTxt("Error parsing argument, object is not a sensor");
    }
    static rs2_internal_t* to_internal(T&& var) { mexLock(); return new rs2_internal_t(new carrier_t(var), carrier_enum::value); }
};

// rs_device.hpp
template<> struct MatlabParamParser::type_traits<rs2::device> { using rs2_internal_t = std::shared_ptr<rs2_device>; };
template<> struct MatlabParamParser::type_traits<rs2::device_list> {
    using rs2_internal_t = std::shared_ptr<rs2_device_list>;
    using use_cells = std::true_type;
};

// rs_record_playback.hpp
template<> struct MatlabParamParser::type_traits<rs2::playback> : MatlabParamParser::type_traits<rs2::device> {
    static rs2::playback from_internal(rs2_internal_t * ptr) { return traits_trampoline::from_internal<rs2::device>(ptr); }
};
template<> struct MatlabParamParser::type_traits<rs2::recorder> : MatlabParamParser::type_traits<rs2::device> {
    static rs2::recorder from_internal(rs2_internal_t * ptr) { return traits_trampoline::from_internal<rs2::device>(ptr); }
};

// rs_processing.hpp
template <typename T> struct over_wrapper {
    using rs2_internal_t = std::shared_ptr<T>;
    static T from_internal(rs2_internal_t * ptr) { return T(**ptr); }
    static rs2_internal_t* to_internal(T&& val) { mexLock(); return new rs2_internal_t(new T(val)); }
};
template <typename T, MatlabParamParser::type_traits<rs2::options>::carrier::types E> struct options_over_wrapper
    : MatlabParamParser::type_traits<rs2::options> {
    using carrier_t = std::shared_ptr<T>;
    using carrier_enum = std::integral_constant<rs2_internal_t::types, E>;
    static T from_internal(rs2_internal_t * ptr) {
        if (ptr->type == carrier_enum::value) return T(**static_cast<carrier_t*>(ptr->ptr));
        mexErrMsgTxt("Error parsing argument, wrong branch of rs2::options inheritance");
    }
    static rs2_internal_t* to_internal(T&& var) { mexLock(); return new rs2_internal_t(new carrier_t(new T(var)), carrier_enum::value); }
};
template<> struct MatlabParamParser::type_traits<rs2::align> : over_wrapper<rs2::align> {};
template<> struct MatlabParamParser::type_traits<rs2::colorizer>
    : options_over_wrapper<rs2::colorizer, MatlabParamParser::type_traits<rs2::options>::carrier::types::rs2_colorizer> {};
template<> struct MatlabParamParser::type_traits<rs2::syncer> : over_wrapper<rs2::syncer> {};
template<> struct MatlabParamParser::type_traits<rs2::frame_queue> : over_wrapper<rs2::frame_queue> {};
template<> struct MatlabParamParser::type_traits<rs2::pointcloud>
    : options_over_wrapper<rs2::pointcloud, MatlabParamParser::type_traits<rs2::options>::carrier::types::rs2_pointcloud> {};


template<> struct MatlabParamParser::type_traits<rs2::process_interface> : type_traits<rs2::options> {
    using carrier_t = std::shared_ptr<rs2::process_interface>;
    using carrier_enum = std::integral_constant<rs2_internal_t::types, rs2_internal_t::types::rs2_process_interface>;
};
template <typename T> struct MatlabParamParser::type_traits<T, typename std::enable_if<std::is_base_of<rs2::process_interface, T>::value>::type>
    : MatlabParamParser::type_traits<rs2::process_interface> {
    static T from_internal(rs2_internal_t * ptr) { mexErrMsgTxt("from_internal<rs2::process_interface>(): This shouldn't happen"); }
    static rs2_internal_t* to_internal(T&& var) { mexLock(); return new rs2_internal_t(new carrier_t(new T(var)), carrier_enum::value); }
};

// rs_context.hpp
// rs2::event_information                       [?]
template<> struct MatlabParamParser::type_traits<rs2::context> { using rs2_internal_t = std::shared_ptr<rs2_context>; };
template<> struct MatlabParamParser::type_traits<rs2::device_hub> { using rs2_internal_t = std::shared_ptr<rs2_device_hub>; };

// rs_pipeline.hpp
template<> struct MatlabParamParser::type_traits<rs2::pipeline> { using rs2_internal_t = std::shared_ptr<rs2_pipeline>; };
template<> struct MatlabParamParser::type_traits<rs2::config> { using rs2_internal_t = std::shared_ptr<rs2_config>; };
template<> struct MatlabParamParser::type_traits<rs2::pipeline_profile> { using rs2_internal_t = std::shared_ptr<rs2_pipeline_profile>; };

// This needs to go at the bottom so that all the relevant type_traits specializations will have already occured.
MatlabParamParser::type_traits<rs2::options>::carrier::~carrier() {
    switch (type) {
    case types::rs2_sensor: delete reinterpret_cast<type_traits<rs2::sensor>::carrier_t*>(ptr);
    case types::rs2_process_interface: delete reinterpret_cast<type_traits<rs2::process_interface>::carrier_t*>(ptr);
    case types::rs2_colorizer: delete reinterpret_cast<type_traits<rs2::colorizer>::carrier_t*>(ptr);
    case types::rs2_pointcloud: delete reinterpret_cast<type_traits<rs2::pointcloud>::carrier_t*>(ptr);
    }
}

rs2::options MatlabParamParser::type_traits<rs2::options>::from_internal(rs2_internal_t * ptr) {
    switch (ptr->type) {
    case carrier::types::rs2_sensor: return traits_trampoline::from_internal<rs2::sensor>(ptr).as<rs2::options>();
    case carrier::types::rs2_process_interface: return *std::shared_ptr<rs2::options>(*static_cast<type_traits<rs2::process_interface>::carrier_t*>(ptr->ptr));
    case carrier::types::rs2_colorizer: return *std::shared_ptr<rs2::options>(*static_cast<type_traits<rs2::colorizer>::carrier_t*>(ptr->ptr));
    case carrier::types::rs2_pointcloud: return *std::shared_ptr<rs2::options>(*static_cast<type_traits<rs2::pointcloud>::carrier_t*>(ptr->ptr));
    default: mexErrMsgTxt("Error parsing argument of type rs2::options: unrecognized carrier type");
    }

}
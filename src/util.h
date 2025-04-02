//
// Created by mijiao on 25-3-31.
//

#ifndef RCLGD_UTIL_H
#define RCLGD_UTIL_H

#include <dlfcn.h>
#include <iostream>
#include <string>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <godot_cpp/variant/string_name.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/typed_dictionary.hpp>

#include <godot_cpp/core/error_macros.hpp>

String get_type_name(uint8_t type_id) {
    switch (type_id) {
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
            return "bool";
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
            return "byte";
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
            return "char";
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
            return "float";
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
            return "double";
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
            return "long double";
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
            return "int8";
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
            return "int16";
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
            return "int32";
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
            return "int64";
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
            return "uint8";
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
            return "uint16";
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
            return "uint32";
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
            return "uint64";
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
            return "string";
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
            return "wchar";
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
            return "wstring";
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
            return "nested_msg";
        default:
            return "unknown";
    }
}

int get_type_length(uint8_t type_id) {
    switch (type_id) {
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
            return 1;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
            return 1;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
            return 1;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
            return 4;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
            return 8;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
            return 10;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
            return 1;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
            return 2;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
            return 4;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
            return 8;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
            return 1;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
            return 2;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
            return 4;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
            return 8;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
            return 8;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
            return 2;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
            return 8;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
            return 0;
        default:
            print_error("Type:", type_id, "not support!");
            return 0;
    }
}

Dictionary analyze_message(const rosidl_message_type_support_t *ts, int indent = 0) {
    Dictionary profile;
    const auto *members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(ts->data);
    print_line(String(" ").repeat(indent), members->size_of_);
    print_line(String("{").indent(String(" ").repeat(indent)));
    if (members->member_count_ > 0) {
        for (uint32_t i = 0; i < members->member_count_; ++i) {
            Dictionary data_info;
            auto member = members->members_[i];
            String array_info;

            // 处理数组类型
            if (member.is_array_) {
                if (member.array_size_ > 0) {
                    array_info = "[" + UtilityFunctions::str(member.array_size_) + "]";
                } else {
                    array_info = "[]";
                }
            }

            // 基本类型信息
            print_line(String("  ").indent(String(" ").repeat(indent)),
                       member.name_, array_info, " : ",
                       get_type_name(member.type_id_), " ", member.offset_, " ");

            data_info["type_id"] = member.type_id_;
            data_info["is_array"] = member.is_array_;
            if (member.is_array_) {
                data_info["array_size"] = member.array_size_;
            }
            if (member.type_id_ == 16 || member.type_id_ == 17) {
                data_info["string_upper_bound"] = member.string_upper_bound_;
            }

            if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
                Dictionary child_msg_profile = analyze_message(member.members_, indent + 2);
                data_info["child_msg"] = child_msg_profile;
            }

            profile[member.name_] = data_info;
        }
        print_line(String("}").indent(String(" ").repeat(indent)));

    }
    return profile;
}

Dictionary get_message_structure(const String &full_msg_type) {
    // 解析包名和消息名
    PackedStringArray msg_type = full_msg_type.split("/", false);
    if (msg_type.size() != 3) {
        print_error("Invalid message format: ", full_msg_type);
        return {};
    }

    String package_name = msg_type[0];
    String type_name = msg_type[2];

    // 获取类型支持库路径
    String lib_name = "lib" + package_name + "__rosidl_typesupport_introspection_cpp";
    String package_prefix;
    try {
        package_prefix = ament_index_cpp::get_package_prefix(package_name.ascii().ptr()).c_str();
    } catch (const ament_index_cpp::PackageNotFoundError &e) {
        print_error("Package not found: ", e.what());
        return {};
    }

    // 动态加载类型支持库
    String lib_path = package_prefix + "/lib/" + lib_name + ".so";
    void *handle = dlopen(lib_path.ascii().ptr(), RTLD_LAZY);
    if (!handle) {
        print_error("Failed to load library: ", dlerror());
        return {};
    }

    // 获取类型支持句柄
    String symbol_name =
            "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" +
            package_name + "__msg__" + type_name;
    auto get_handle = reinterpret_cast<const rosidl_message_type_support_t *(*)()>(dlsym(handle,
                                                                                         symbol_name.ascii().ptr()));

    if (!get_handle) {
        print_error("Symbol not found: ", dlerror());
        dlclose(handle);
        return {};
    }

    const rosidl_message_type_support_t *ts = get_handle();
    if (!ts) {
        print_error("Invalid typesupport identifier");
        dlclose(handle);
        return {};
    }

    print_line("Message structure for [", full_msg_type, "]");

    Dictionary ret;
    ret = analyze_message(ts);

    dlclose(handle);
    return ret;
}

Dictionary bin_to_msg(const PackedByteArray &bin, const Dictionary &msg_info, int64_t &offset);

Variant bin_to_single_msg(const PackedByteArray &bin, const Dictionary &msg_detail, int64_t &offset) {
    int type_id = int(msg_detail.get("type_id", nullptr));
    if (type_id == 18) {
        Dictionary child_msg = msg_detail.get("child_msg", {});
        return bin_to_msg(bin, child_msg, offset);
    } else if (type_id == 16 || type_id == 17) {
        String s;
        auto size = bin.decode_u32(offset);
        offset += 4;
        s.resize(size);
        for (int i = 0; i < size; i++) {
            s[i] = bin.decode_u8(offset);
            offset += 1;
        }
        if (size % 4 != 0) {
            offset += 4 - size % 4;
        }
        if ((size - 1) % 8 >= 4){
            offset += 4;
        }
        return s;
    } else {
        switch (type_id) {
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL: {
                bool ret = bin.decode_u8(offset);
                offset += 1;
                return ret;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE: {
                auto ret = bin.decode_u8(offset);
                offset += 1;
                return ret;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR: {
                auto ret = bin.decode_s8(offset);
                offset += 1;
                return ret;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT: {
                auto ret = bin.decode_float(offset);
                offset += 4;
                return ret;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE: {
                auto ret = bin.decode_double(offset);
                offset += 8;
                return ret;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE: {
                print_error("Type:", type_id, "not support!");
                offset += 12;
                return 0;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8: {
                auto ret = bin.decode_s8(offset);
                offset += 1;
                return ret;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16: {
                auto ret = bin.decode_s16(offset);
                offset += 2;
                return ret;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32: {
                auto ret = bin.decode_s32(offset);
                offset += 4;
                return ret;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64: {
                auto ret = bin.decode_s64(offset);
                offset += 8;
                return ret;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8: {
                auto ret = bin.decode_u8(offset);
                offset += 1;
                return ret;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16: {
                auto ret = bin.decode_u16(offset);
                offset += 2;
                return ret;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32: {
                auto ret = bin.decode_s32(offset);
                offset += 4;
                return ret;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64: {
                auto ret = bin.decode_u32(offset);
                offset += 4;
                return ret;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR: {
                auto ret = bin.decode_u32(offset);
                offset += 2;
                return ret;
            }
            default:
                print_error("Type:", type_id, "not support!");
                return nullptr;
        }
    }
}

Dictionary bin_to_msg(const PackedByteArray &bin, const Dictionary &msg_info, int64_t &offset) {
    auto keys = msg_info.keys();
    auto values = msg_info.values();
    Dictionary msg;
    for (int i = 0; i < keys.size(); i++) {
        String name = keys.get(i);
        Dictionary msg_detail = values.get(i);
        if (msg_detail.get("is_array", nullptr)) {
            Array a;
            if (size_t(msg_detail["array_size"]) != 0) {
                a.resize(msg_detail["array_size"]);
            } else {
                a.resize(bin.decode_u64(offset));
                offset += 8;
            }
            for (int j = 0; j < a.size(); j++) {
                a[j] = bin_to_single_msg(bin, msg_detail, offset);
            }
            msg[name] = a;
        } else {
            msg[name] = bin_to_single_msg(bin, msg_detail, offset);
        }
    }
    return msg;
}

void msg_to_bin(const Dictionary &msg, const Dictionary &msg_info, int64_t &offset, PackedByteArray &ret);

void single_msg_to_bin(const Variant &msg, const Dictionary &msg_detail, int64_t &offset, PackedByteArray &ret) {
    int type_id = int(msg_detail.get("type_id", nullptr));
    if (type_id == 18) {
        Dictionary child_msg = msg_detail.get("child_msg", {});
        msg_to_bin(msg, child_msg, offset, ret);
    } else if (type_id == 16 || type_id == 17) {
        String s(msg);
        if(s.is_empty()){
            ret.resize(ret.size() + 8);
            ret.encode_s32(offset, 1);
            offset += 8;
        }else{
            ret.resize(ret.size() + 4);
            ret.encode_s32(offset, s.length() + 1);
            offset += 4;
            for (int i = 0; i <= s.length(); i++) {
                ret.resize(ret.size() + 1);
                ret.encode_u8(offset, s[i]);
                offset += 1;
            }
            if (s.length() % 4 != 0) {
                offset += 4 - (s.length()+ 1) % 4;
                ret.resize(offset);
            }
        }
    } else {
        auto n = ret.size() % 4;
        ret.resize(ret.size() + n);
        offset += n;
        switch (type_id) {
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL: {
                ret.resize(ret.size() + 1);
                ret.encode_u8(offset, msg);
                offset += 1;
                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE: {
                ret.resize(ret.size() + 1);
                ret.encode_u8(offset, msg);
                offset += 1;
                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR: {
                ret.resize(ret.size() + 1);
                ret.encode_u8(offset, msg);
                offset += 1;
                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT: {
                ret.resize(ret.size() + 4);
                ret.encode_float(offset, msg);
                offset += 4;
                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE: {
                ret.resize(ret.size() + 8);
                ret.encode_double(offset, msg);
                offset += 8;
                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE: {
                ret.resize(ret.size() + 8);
                print_error("Type:", type_id, "not support!");
                offset += 10;
                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8: {
                ret.resize(ret.size() + 1);
                ret.encode_s8(offset, msg);
                offset += 1;
                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16: {
                ret.resize(ret.size() + 2);
                ret.encode_s16(offset, msg);
                offset += 2;
                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32: {
                ret.resize(ret.size() + 4);
                ret.encode_s32(offset, msg);
                offset += 4;
                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64: {
                ret.resize(ret.size() + 8);
                ret.encode_s64(offset, msg);
                offset += 8;
                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8: {
                ret.resize(ret.size() + 1);
                ret.encode_u8(offset, msg);
                offset += 1;
                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16: {
                ret.resize(ret.size() + 2);
                ret.encode_u16(offset, msg);
                offset += 2;
                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32: {
                ret.resize(ret.size() + 4);
                ret.encode_u32(offset, msg);
                offset += 4;
                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64: {
                ret.resize(ret.size() + 8);
                ret.encode_u64(offset, msg);
                offset += 8;
                break;
            }
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR: {
                ret.resize(ret.size() + 2);
                ret.encode_u16(offset, msg);
                offset += 2;
                break;
            }
            default:
                print_error("Type:", type_id, "not support!");
        }
    }
}


void msg_to_bin(const Dictionary &msg, const Dictionary &msg_info, int64_t &offset, PackedByteArray &ret) {
    auto keys = msg_info.keys();
    auto values = msg_info.values();
    for (int i = 0; i < keys.size(); i++) {
        String name = keys.get(i);
        Dictionary msg_detail = values.get(i);
        if (msg_detail.get("is_array", nullptr)) {
            Array a = msg[name];
            if (size_t(msg_detail["array_size"]) != 0) {
                for (int64_t j = 0; j < a.size(); j++) {
                    single_msg_to_bin(a[j], msg_detail, offset, ret);
                }
                for (size_t j = a.size(); j < size_t(msg_detail["array_size"]); j++) {
                    single_msg_to_bin(nullptr, msg_detail, offset, ret);
                }
            } else {
                ret.resize(ret.size() + 8);
                ret.encode_u64(offset, a.size());
                offset += 8;
                for (int64_t j = 0; j < a.size(); j++) {
                    single_msg_to_bin(a[j], msg_detail, offset, ret);
                }
            }
        } else {
            single_msg_to_bin(msg[name], msg_detail, offset, ret);
        }
    }
}


#endif //RCLGD_UTIL_H

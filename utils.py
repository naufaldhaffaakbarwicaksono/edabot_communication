def ros_message_to_dict(msg):
    result = {}
    for field_name in msg.get_fields_and_field_types():
        value = getattr(msg, field_name)

        if hasattr(value, "get_fields_and_field_types"):
            result[field_name] = ros_message_to_dict(value)

        elif isinstance(value, (list, tuple)):
            result[field_name] = [
                ros_message_to_dict(item)
                if hasattr(item, "get_fields_and_field_types")
                else item
                for item in value
            ]
        else:
            result[field_name] = value
    return result

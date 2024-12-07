def ros_msg_to_dict(msg):
    result = {}
    for field_name, field_type in msg.get_fields_and_field_types().items():
        value = getattr(msg, field_name)

        # skip stamp and header msg
        if field_name == "stamp" or field_name == "header":
            continue

        # Check if the field is a nested message
        if hasattr(value, "get_fields_and_field_types"):
            result[field_name] = ros_msg_to_dict(value)

        # Check if the field is a list of nested messages
        elif isinstance(value, (list, tuple)):
            result[field_name] = [
                ros_msg_to_dict(item)
                if hasattr(item, "get_fields_and_field_types")
                else item
                for item in value
            ]

        # Basic field assignment
        else:
            result[field_name] = value
    return result


def dict_to_ros_msg(msg_type, data):
    # Create an instance of the ROS message
    msg = msg_type()

    # Populate fields using get_fields_and_field_types()
    for field_name, field_type in msg.get_fields_and_field_types().items():
        if field_name in data:
            value = data[field_name]

            # Check if the field is a nested message
            if hasattr(getattr(msg, field_name), "get_fields_and_field_types"):
                nested_msg_type = type(getattr(msg, field_name))
                setattr(msg, field_name, dict_to_ros_msg(nested_msg_type, value))

            # Check if the field is a list of nested messages
            elif isinstance(value, list) and "[]" in field_type:
                element_type = (
                    type(getattr(msg, field_name)[0])
                    if getattr(msg, field_name)
                    else None
                )
                if element_type:
                    setattr(
                        msg,
                        field_name,
                        [
                            dict_to_ros_msg(element_type, item)
                            if isinstance(item, dict)
                            else item
                            for item in value
                        ],
                    )
                else:
                    setattr(msg, field_name, value)

            # Basic field assignment
            else:
                setattr(msg, field_name, value)

    return msg

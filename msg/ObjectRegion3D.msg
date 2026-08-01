# Defines a object with associated region in 3D space

# The detected 3D object. Its own header (or the ObjectRegion3DArray header,
# if the detection does not carry one) is authoritative for frame/stamp.
vision_msgs/Detection3D object

# Human-readable class name resolved from the object's hypothesis class_id,
# which is left untouched.
string class_name

# The name of the region associated with the object
string region
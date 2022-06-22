# Msg from rr_collision_safety

# if collision is possible or not
bool is_collide

# obstacle의 course 정보
# course must be in [1, 2]
#   1 : in_course
#   2 : out_course

# obstacle의 area 정보
# area must be in [1, 2, 3]
#   1 : Front
#   2 : Middle
#   3 : Back

# (1, 1), (1, 2) ~ (2, 3)
bool[6] collision_space
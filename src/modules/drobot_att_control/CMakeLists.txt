px4_add_module(
    MODULE modules__drobot_att_control
    MAIN drobot_att_control
    STACK_MAIN 2000
    SRCS
        drobot_control_main.cpp
    DEPENDS
        px4_work_queue
)

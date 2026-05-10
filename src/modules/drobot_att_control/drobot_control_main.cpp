#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

extern "C" __EXPORT int drobot_att_control_main(int argc, char *argv[]);

class DrobotControl : public ModuleBase, public px4::ScheduledWorkItem
{
public:
    static Descriptor desc;

    DrobotControl() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl) {}
    ~DrobotControl() override = default;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

private:
    void Run() override;
    void update_vehicle_type(uint8_t type);
    void stop_motors();
    void set_servos(float position);

    // 구독: 상위 제어기 명령 및 현재 상태
    uORB::Subscription _vcmd_sub{ORB_ID(vehicle_command)};
    uORB::Subscription _vstatus_sub{ORB_ID(vehicle_status)};

    // 발행: 기체 타입 변경, 서보 제어, 강제 정지용
    uORB::Publication<vehicle_status_s> _vstatus_pub{ORB_ID(vehicle_status)};
    uORB::Publication<actuator_servos_s> _servo_pub{ORB_ID(actuator_servos)};
    uORB::Publication<vehicle_thrust_setpoint_s> _thrust_pub{ORB_ID(vehicle_thrust_setpoint)};
    uORB::Publication<vehicle_torque_setpoint_s> _torque_pub{ORB_ID(vehicle_torque_setpoint)};

    enum class State {
        ROVER,
        TRANSITION_TO_DRONE,
        DRONE,
        TRANSITION_TO_ROVER
    } _state{State::ROVER};
    hrt_abstime _transition_start{0};
    const hrt_abstime _transition_delay{2000000}; // 2초 (2,000,000 us)
};

bool DrobotControl::init()
{
    ScheduleOnInterval(20_ms); // $50Hz$ 주기로 실행
    return true;
}

void DrobotControl::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup(desc);
        return;
    }

    // 1. 상위 제어기(RPi)로부터의 모드 전환 명령 수신
    vehicle_command_s vcmd;
    if (_vcmd_sub.update(&vcmd)) {
        if (vcmd.command == vehicle_command_s::VEHICLE_CMD_CUSTOM_0) {
            if (vcmd.param1 > 0.5f && _state == State::ROVER) {
                _state = State::TRANSITION_TO_DRONE;
                _transition_start = hrt_absolute_time();
            } else if (vcmd.param1 < 0.5f && _state == State::DRONE) {
                _state = State::TRANSITION_TO_ROVER;
                _transition_start = hrt_absolute_time();
            }
        }
    }

    // 2. 상태 머신 로직
    switch (_state) {
    case State::ROVER:
        set_servos(-1.0f); // 로버 모드 위치 (0 deg)
        break;

    case State::DRONE:
        set_servos(1.0f); // 드론 모드 위치 (90 deg)
        break;

    case State::TRANSITION_TO_DRONE:
        stop_motors();    // 트랜지션 중 안전을 위한 출력 차단
        set_servos(1.0f); // 서보 구동 시작
        if (hrt_absolute_time() - _transition_start > _transition_delay) {
            _state = State::DRONE;
        }
        break;

    case State::TRANSITION_TO_ROVER:
        stop_motors();     // 트랜지션 중 안전을 위한 출력 차단
        set_servos(-1.0f); // 서보 구동 시작
        if (hrt_absolute_time() - _transition_start > _transition_delay) {
            _state = State::ROVER;
        }
        break;
    }

    // 3. vehicle_type 발행: 변경 시 + 반영 안 됐으면 재발행
    uint8_t desired_type = (_state == State::DRONE || _state == State::TRANSITION_TO_DRONE)
                           ? vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
                           : vehicle_status_s::VEHICLE_TYPE_ROVER;

    vehicle_status_s status;
    if (_vstatus_sub.copy(&status) && status.vehicle_type != desired_type) {
        update_vehicle_type(desired_type);
    }
}

void DrobotControl::update_vehicle_type(uint8_t type)
{
    vehicle_status_s status;
    if (_vstatus_sub.copy(&status)) {
        if (status.vehicle_type != type) {
            status.vehicle_type = type;
            status.timestamp = hrt_absolute_time();
            _vstatus_pub.publish(status);
        }
    }
}

void DrobotControl::stop_motors()
{
    vehicle_thrust_setpoint_s thrust{};
    vehicle_torque_setpoint_s torque{};
    thrust.timestamp = hrt_absolute_time();
    torque.timestamp = hrt_absolute_time();
    _thrust_pub.publish(thrust);
    _torque_pub.publish(torque);
}

void DrobotControl::set_servos(float position)
{
    actuator_servos_s servos{};
    servos.timestamp = hrt_absolute_time();
    for (int i = 0; i < 4; i++) {
        servos.control[i] = position;
    }
    _servo_pub.publish(servos);
}

int DrobotControl::task_spawn(int argc, char *argv[])
{
    DrobotControl *instance = new DrobotControl();
    if (instance) {
        desc.object.store(instance);
        desc.task_id = task_id_is_work_queue;
        if (instance->init()) {
            return PX4_OK;
        }
    } else {
        PX4_ERR("alloc failed");
    }
    delete instance;
    desc.object.store(nullptr);
    desc.task_id = -1;
    return PX4_ERROR;
}

int DrobotControl::custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
int DrobotControl::print_usage(const char *reason) { return 0; }

ModuleBase::Descriptor DrobotControl::desc{
    DrobotControl::task_spawn,
    DrobotControl::custom_command,
    DrobotControl::print_usage,
};

int drobot_att_control_main(int argc, char *argv[])
{
    return ModuleBase::main(DrobotControl::desc, argc, argv);
}

#include "chassis.hpp"
#include "robot_config.hpp"
#include "djimotors.hpp"
#include "motorservice.hpp"
#include "pid.hpp"
#include "constants.hpp"
#include "remoter.hpp"
#include "msg.hpp"
#include "tx_api.h"

#include <array>
#include <cmath>


namespace chassis
{
namespace
{

constexpr std::size_t module_count = 4U;
constexpr float wheel_radius = 0.066f;
constexpr float module_radius = 0.280014f;
constexpr float inverse_sqrt_two = 0.7071067812f;
constexpr float yaw_to_module_component = module_radius * inverse_sqrt_two;
constexpr float half_pi = math::pi * 0.5f;
constexpr float stopped_epsilon = 1.0e-6f;

// Manual input limits follow the legacy RM26 linear-speed scale and spin rate.
constexpr float manual_linear_speed_limit = 2.0f;
constexpr float manual_angular_speed_limit = 5.0f;
constexpr float manual_axis_deadzone = 0.05f;

float patrol_angular_velocity = 5.0f;
constexpr float patrol_max = 9.0f;
constexpr float patrol_min = 4.0f;
static_assert(patrol_max > patrol_min);

constexpr ULONG control_period_ticks = 1U;
constexpr UINT chassis_thread_priority = 9U;
constexpr std::size_t chassis_stack_size = 4096U;

// Maximum interval without a new remoter state: 200 ms.
constexpr ULONG remoter_message_timeout_ticks = TX_TIMER_TICKS_PER_SECOND / 5U;

// Module order is fixed throughout the controller: RF, LF, LR, RR.
constexpr std::array<float, module_count> steer_offsets{
    2.868f,
    1.287f,
    0.761f,
    -2.888f,
};

// Mechanical installation direction inherited from RM26_H7.
constexpr std::array<float, module_count> drive_directions{
    1.0f,
    -1.0f,
    -1.0f,
    1.0f,
};

control::pid RFdrive_pid{500.0f, 0.0f, 0.0f, 10000.0f, 0.0f};
control::pid LFdrive_pid{500.0f, 0.0f, 0.0f, 10000.0f, 0.0f};
control::pid LRdrive_pid{500.0f, 0.0f, 0.0f, 10000.0f, 0.0f};
control::pid RRdrive_pid{500.0f, 0.0f, 0.0f, 10000.0f, 0.0f};
control::pid RFsteer_position_pid{40.0f, 0.0f, 0.0f, 10000.0f, 0.0f};
control::pid LFsteer_position_pid{40.0f, 0.0f, 0.0f, 10000.0f, 0.0f};
control::pid LRsteer_position_pid{40.0f, 0.0f, 0.0f, 10000.0f, 0.0f};
control::pid RRsteer_position_pid{40.0f, 0.0f, 0.0f, 10000.0f, 0.0f};
control::pid RFsteer_velocity_pid{1000.0f, 5.0f, 200.0f, 10000.0f, 3000.0f};
control::pid LFsteer_velocity_pid{1000.0f, 5.0f, 200.0f, 10000.0f, 3000.0f};
control::pid LRsteer_velocity_pid{1000.0f, 5.0f, 200.0f, 10000.0f, 3000.0f};
control::pid RRsteer_velocity_pid{1000.0f, 5.0f, 200.0f, 10000.0f, 3000.0f};

const std::array<control::pid*, module_count> drive_pids
{
    &RFdrive_pid,
    &LFdrive_pid,
    &LRdrive_pid,
    &RRdrive_pid,
};

const std::array<control::pid*, module_count> steer_position_pids
{
    &RFsteer_position_pid,
    &LFsteer_position_pid,
    &LRsteer_position_pid,
    &RRsteer_position_pid,
};

const std::array<control::pid*, module_count> steer_velocity_pids
{
    &RFsteer_velocity_pid,
    &LFsteer_velocity_pid,
    &LRsteer_velocity_pid,
    &RRsteer_velocity_pid,
};


static_assert(robot::motors::RFsteerMotor_model == robot::motors::model::dji_gm6020);
static_assert(robot::motors::LFsteerMotor_model == robot::motors::model::dji_gm6020);
static_assert(robot::motors::LRsteerMotor_model == robot::motors::model::dji_gm6020);
static_assert(robot::motors::RRsteerMotor_model == robot::motors::model::dji_gm6020);
static_assert(robot::motors::RFdriveMotor_model == robot::motors::model::dji_m3508);
static_assert(robot::motors::LFdriveMotor_model == robot::motors::model::dji_m3508);
static_assert(robot::motors::LRdriveMotor_model == robot::motors::model::dji_m3508);
static_assert(robot::motors::RRdriveMotor_model == robot::motors::model::dji_m3508);

motors::gm6020 RFsteerMotor{robot::motors::RFsteerMotor};
motors::gm6020 LFsteerMotor{robot::motors::LFsteerMotor};
motors::gm6020 LRsteerMotor{robot::motors::LRsteerMotor};
motors::gm6020 RRsteerMotor{robot::motors::RRsteerMotor};
motors::m3508 RFdriveMotor{robot::motors::RFdriveMotor};
motors::m3508 LFdriveMotor{robot::motors::LFdriveMotor};
motors::m3508 LRdriveMotor{robot::motors::LRdriveMotor};
motors::m3508 RRdriveMotor{robot::motors::RRdriveMotor};

const std::array<motors::gm6020*, module_count> steer_motors{
    &RFsteerMotor,
    &LFsteerMotor,
    &LRsteerMotor,
    &RRsteerMotor,
};

const std::array<motors::m3508*, module_count> drive_motors{
    &RFdriveMotor,
    &LFdriveMotor,
    &LRdriveMotor,
    &RRdriveMotor,
};


motors::motor_service service{};
bool register_motors() noexcept
{
    bool all_registered = true;

    for(motors::gm6020* motor : steer_motors)
    {
        const bool registered = service.register_motor(*motor);
        all_registered = registered && all_registered;
    }

    for(motors::m3508* motor : drive_motors)
    {
        const bool registered = service.register_motor(*motor);
        all_registered = registered && all_registered;
    }
    return all_registered;
}

template <typename... Omegas>
float patrol_angular_velocity_multisine(ULONG tick, float last_w,
                                        float downwards_coeff,
                                        Omegas... omegas) noexcept
{
    // Reserved for a future continuity/rate-limit stage.
    (void)last_w;

    constexpr std::size_t term_count = sizeof...(Omegas);
    constexpr float midpoint = (patrol_max + patrol_min) * 0.5f;
    constexpr float half_range = (patrol_max - patrol_min) * 0.5f;

    if constexpr (term_count == 0U)
    {
        return midpoint;
    }

    // A decreasing geometric envelope requires 0 < coefficient <= 1.
    if (!std::isfinite(downwards_coeff) ||
        downwards_coeff <= 0.0f || downwards_coeff > 1.0f)
    {
        return midpoint;
    }

    const std::array<float, term_count> angular_frequencies{
        static_cast<float>(omegas)...,
    };
    const double time_seconds = static_cast<double>(tick) / TX_TIMER_TICKS_PER_SECOND;  // *0.001

    float weighted_sum = 0.0f;
    float amplitude_sum = 0.0f;
    float amplitude = downwards_coeff;
    for (const float omega : angular_frequencies)
    {
        if (!std::isfinite(omega))
        {
            return midpoint;
        }

        const double phase = std::fmod(
            static_cast<double>(omega) * time_seconds,
            static_cast<double>(math::two_pi));
        weighted_sum += amplitude * static_cast<float>(std::sin(phase));
        amplitude_sum += amplitude;
        amplitude *= downwards_coeff;
    }

    if (amplitude_sum <= 0.0f || !std::isfinite(weighted_sum))
    {
        return midpoint;
    }

    const float normalized = std::fmax(
        -1.0f, std::fmin(1.0f, weighted_sum / amplitude_sum));
    return midpoint + half_range * normalized;
}

float get_stairs(double time, float period){
    if (period <= 0 || time <= 0) return 0;
    float remnant = time;
    remnant=std::fmod(time, period);  // std::fmod(被除数, 除数)
    return (remnant > period/2) ? 1.0f : -1.0f; 
}

template <typename... Periods>
float patrol_angular_velocity_stairs(ULONG tick, float last_w,
                                        float downwards_coeff,
                                        Periods... periods) noexcept
{
    (void)last_w;
    constexpr std::size_t term_count = sizeof...(periods);
    constexpr float midpoint = (patrol_max + patrol_min) * 0.5f;
    constexpr float half_range = (patrol_max - patrol_min) * 0.5f;

    if constexpr (term_count == 0U)
    {
        return midpoint;
    }

    // A decreasing geometric envelope requires 0 < coefficient <= 1.
    if (!std::isfinite(downwards_coeff) ||
        downwards_coeff <= 0.0f || downwards_coeff > 1.0f)
    {
        return midpoint;
    }

    const std::array<float, term_count> angular_frequencies{
        static_cast<float>(periods)...,
    };
    const double time_seconds = static_cast<double>(tick) / TX_TIMER_TICKS_PER_SECOND;

    float weighted_sum = 0.0f;
    float amplitude_sum = 0.0f;
    float amplitude = downwards_coeff;
    for (const float period : angular_frequencies)
    {
        if (!std::isfinite(period))
        {
            return midpoint;
        }

        // const double phase = std::fmod(
        //     static_cast<double>(period) * time_seconds,
        //     static_cast<double>(math::two_pi));
        weighted_sum += amplitude * static_cast<float>(get_stairs(time_seconds, period));
        amplitude_sum += amplitude;
        amplitude *= downwards_coeff;
    }

    if (amplitude_sum <= 0.0f || !std::isfinite(weighted_sum))
    {
        return midpoint;
    }

    const float normalized = std::fmax(
        -1.0f, std::fmin(1.0f, weighted_sum / amplitude_sum));
    return midpoint + half_range * normalized;
}

struct planar_velocity
{
    float x;
    float y;
};

struct optimized_target
{
    float steer_position;
    float drive_velocity;
};

float wrap_to_pi(float angle) noexcept
{
    while(angle > math::pi)
    {
        angle -= math::two_pi;
    }
    while(angle <= -math::pi)
    {
        angle += math::two_pi;
    }
    return angle;
}

float finite_or_zero(float value) noexcept
{
    return std::isfinite(value) ? value : 0.0f;
}

optimized_target optimize_target(float current_encoder_position, float ideal_wheel_angle, float steer_offset, float drive_velocity) noexcept
{
    const float current_wheel_angle = wrap_to_pi(current_encoder_position - steer_offset);
    float angle_error = wrap_to_pi(ideal_wheel_angle - current_wheel_angle);

    if(angle_error > half_pi)
    {
        angle_error -= math::pi;
        drive_velocity = -drive_velocity;
    }
    else if(angle_error < -half_pi)
    {
        angle_error += math::pi;
        drive_velocity = -drive_velocity;
    }
    return {current_encoder_position + angle_error, drive_velocity};
}

void apply_control(chassis::velocity_command command) noexcept
{
    command.vx = finite_or_zero(command.vx);
    command.vy = finite_or_zero(command.vy);
    command.vw = finite_or_zero(command.vw);

    const float rotational_component = command.vw * yaw_to_module_component;

    // Rigid-body velocity v_i = (vx - vw*y_i, vy + vw*x_i), ordered RF/LF/LR/RR.
    // 轮组 x 方向速度 = vx − vw × 轮组的 y 坐标
    // 轮组 y 方向速度 = vy + vw × 轮组的 x 坐标
    const std::array<planar_velocity, module_count> module_velocities{
        planar_velocity{command.vx + rotational_component, command.vy + rotational_component},
        planar_velocity{command.vx - rotational_component, command.vy + rotational_component},
        planar_velocity{command.vx - rotational_component, command.vy - rotational_component},
        planar_velocity{command.vx + rotational_component, command.vy - rotational_component},
    };

    const bool stopped = std::fabs(command.vx) <= stopped_epsilon && std::fabs(command.vy) <= stopped_epsilon && std::fabs(command.vw) <= stopped_epsilon;

    for (std::size_t i = 0; i < module_count; ++i) 
    {
        float steer_reference = steer_motors[i]->fdb.position;
        float drive_reference = 0.0f;

        if(!stopped) 
        {
            const planar_velocity velocity = module_velocities[i];
            const float linear_speed = std::sqrt(velocity.x*velocity.x + velocity.y*velocity.y);
            const float ideal_wheel_angle = std::atan2(velocity.y, velocity.x);

            drive_reference = drive_directions[i] * linear_speed / wheel_radius;
            const optimized_target target = optimize_target(steer_motors[i]->fdb.position, ideal_wheel_angle, steer_offsets[i], drive_reference);
            steer_reference = target.steer_position;
            drive_reference = target.drive_velocity;
        }

        drive_pids[i]->ref = drive_reference;  // 驱动目标角速度
        drive_pids[i]->fdb = drive_motors[i]->fdb.velocity;  // 
        drive_pids[i]->update();
        const float drive_current = drive_pids[i]->result;

        steer_position_pids[i]->ref = steer_reference;
        steer_position_pids[i]->fdb = steer_motors[i]->fdb.position;
        steer_position_pids[i]->update();
        const float steer_velocity_reference = steer_position_pids[i]->result;

        steer_velocity_pids[i]->ref = steer_velocity_reference;
        steer_velocity_pids[i]->fdb = steer_motors[i]->fdb.velocity;
        steer_velocity_pids[i]->update();
        const float steer_current = steer_velocity_pids[i]->result;

        drive_motors[i]->set_current(static_cast<std::int16_t>(drive_current));
        steer_motors[i]->set_current(static_cast<std::int16_t>(steer_current));
    }
}

void relax_all_motors() noexcept
{
    for (motors::gm6020* motor : steer_motors)
    {
        motor->relax();
    }
    for (motors::m3508* motor : drive_motors)
    {
        motor->relax();
    }
}

void reset_all_controllers() noexcept
{
    for (std::size_t i = 0; i < module_count; ++i)
    {
        drive_pids[i]->reset_state(0.0f, drive_motors[i]->fdb.velocity);
        steer_position_pids[i]->reset_state(steer_motors[i]->fdb.position, steer_motors[i]->fdb.position);
        steer_velocity_pids[i]->reset_state(0.0f, steer_motors[i]->fdb.velocity);
    }
}


msg::subscriber remoter_subscriber{};  // 保存遥控器通道的订阅信息
::remoter::state latest_remoter_state{};
ULONG last_remoter_message_tick = 0U;
bool remoter_message_received = false;

void poll_remoter_state() noexcept
{
    ::remoter::state new_state{};
    if (msg::read(remoter_subscriber, new_state) == types::status::ok)  // 通过哪个订阅对象读取数据，将读到的数据写到哪个对象
    {
        latest_remoter_state = new_state;
        last_remoter_message_tick = tx_time_get();
        remoter_message_received = true;
    }

    const ULONG now = tx_time_get();
    const ULONG elapsed = now - last_remoter_message_tick;

    if (!remoter_message_received || elapsed >= remoter_message_timeout_ticks)
    {
        latest_remoter_state = {};
    }
}

bool init_remoter() noexcept
{
    if (::remoter::service::instance().init() != types::status::ok) 
    {
        return false;
    }

    remoter_subscriber = msg::subscribe(::remoter::service::instance().output());  // 取得遥控器状态通道
    return remoter_subscriber.valid();
}

float normalize_manual_axis(float value) noexcept
{
    value = finite_or_zero(value);
    value = std::fmax(-1.0f, std::fmin(1.0f, value));

    const float magnitude = std::fabs(value);
    if (magnitude <= manual_axis_deadzone) 
    {
        return 0.0f;
    }

    const float normalized = (magnitude - manual_axis_deadzone) / (1.0f - manual_axis_deadzone);
    return std::copysign(normalized, value);  // 幅度来源，符号来源
}

chassis::velocity_command manual_command_from_remoter() noexcept
{
    if (latest_remoter_state.offline || 
        latest_remoter_state.active_source != ::remoter::source::dr16 ||
        latest_remoter_state.left_sw != ::remoter::sw_state::mid) 
    {
        return {};
    }

    const float vx = normalize_manual_axis(latest_remoter_state.left_y) *
                     manual_linear_speed_limit;
    const float vy = -normalize_manual_axis(latest_remoter_state.left_x) *
                     manual_linear_speed_limit;
    const float vw = normalize_manual_axis(latest_remoter_state.right_x) *
                     manual_angular_speed_limit;
    return {vx, vy, vw};
}

velocity_command patrol_command_from_remoter() noexcept
{
    if (latest_remoter_state.offline || 
        latest_remoter_state.active_source != ::remoter::source::dr16 ||
        latest_remoter_state.left_sw != ::remoter::sw_state::up) 
    {
        return {};
    }

    switch (latest_remoter_state.right_sw)
    {
        case ::remoter::sw_state::low:
            patrol_angular_velocity = 5.00f;
            break;
        case ::remoter::sw_state::mid:
            patrol_angular_velocity = patrol_angular_velocity_stairs(
            tx_time_get(), patrol_angular_velocity, 0.8f,
            1.0f);
            break;
        case ::remoter::sw_state::up:
            patrol_angular_velocity = patrol_angular_velocity_multisine(
            tx_time_get(), patrol_angular_velocity, 0.8f,
            3.2934857f, 1.534807f, 4.13245f, 1.098234957f);
            break;
        default:
            patrol_angular_velocity = 5.00f;
            break;
    }      
    return {0.0f, 0.0f, patrol_angular_velocity}; 
}

mode requested_mode_from_inputs() noexcept
{
    // DR16 loss, an explicit left-switch Low request, or an unavailable gimbal
    // command channel always forces Relax. Mid and Up are both delegated to
    // the gimbal through Following.
    if (latest_remoter_state.offline ||
        latest_remoter_state.active_source != ::remoter::source::dr16)
    {
        return mode::relax;
    }

    switch (latest_remoter_state.left_sw)
    {
        case ::remoter::sw_state::mid:
            return mode::manual; // with chassis only
        case ::remoter::sw_state::up:
            // return gimbal_control_command_ready() ? mode::following : mode::relax;
            return mode::patrol; // with chassis only
        case ::remoter::sw_state::low:
            return mode::relax; // with chassis only
        default:
            return mode::relax;
    }
}

TX_THREAD chassis_thread{};
alignas(8) std::uint8_t chassis_stack[chassis_stack_size]{};
bool chassis_thread_started = false;

void chassis_thread_entry(ULONG initial_input) 
{
    (void)initial_input;
    relax_all_motors();
    if (! register_motors() || service.start() != types::status::ok)
    {
        return;
    }
    service.send_control();
    if (! init_remoter()) {
        return;
    }
    reset_all_controllers();

    mode current_mode = mode::relax;
    for(;;)
    {
        poll_remoter_state();
        const mode requested_mode = requested_mode_from_inputs();

        if (requested_mode != current_mode)
        {
            relax_all_motors();
            reset_all_controllers();
            current_mode = requested_mode;
        }

        switch (current_mode)
        {
            case mode::relax:
                relax_all_motors();
                break;

            case mode::manual:
                apply_control(manual_command_from_remoter());
                break;

            case mode::patrol:
                apply_control(patrol_command_from_remoter());
                break;

            default:
                relax_all_motors();
                break;
        }
        service.send_control();
        tx_thread_sleep(control_period_ticks);
    }

}

}

void run() noexcept
{
    if (chassis_thread_started)
    {
        return;
    }

    const UINT status = tx_thread_create(
        &chassis_thread,
        const_cast<CHAR*>("chassis"),
        chassis_thread_entry,
        0U,
        chassis_stack,
        sizeof(chassis_stack),
        chassis_thread_priority,
        chassis_thread_priority,
        TX_NO_TIME_SLICE,
        TX_AUTO_START);

    chassis_thread_started = (status == TX_SUCCESS);
    
}

}
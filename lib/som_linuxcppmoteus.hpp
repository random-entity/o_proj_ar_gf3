#pragma once

#include <condition_variable>
#include <map>
#include <memory>

#include "moteus.h"
#ifdef __RASPBERRY_PI__
#include "pi3hat.h"
#endif

#include "slcm_multiplex.hpp"
#include "slcm_parser.hpp"
#include "slcm_printer.hpp"
#include "slcm_utils.hpp"

#ifdef NaN
#undef NaN
#endif
#define NaN (std::numeric_limits<double>::quiet_NaN())

using namespace mjbots;

namespace som {

class ServoSystem {
 protected:
  class Servo {
    friend class ServoSystem;

   public:
    Servo(const int id, const int bus,
          const std::shared_ptr<moteus::Transport>& transport,
          const CmdPosRelTo usr_cmd_pos_rel_to,
          const RplPosRelTo usr_rpl_pos_rel_to,
          const moteus::PositionMode::Format& cmd_fmt,
          const moteus::PositionMode::Command& init_cmd,
          const moteus::Query::Format& rpl_fmt, const bool use_aux2,
          const RplPosRelTo usr_rpl_aux2_pos_rel_to)
        : controller_{std::make_unique<moteus::Controller>([&]() {
            moteus::Controller::Options options;
            options.id = id;
            options.bus = bus;
            options.transport = transport;
            options.position_format = cmd_fmt;
            options.query_format = rpl_fmt;
            return options;
          }())},
          usr_cmd_pos_rel_to_{usr_cmd_pos_rel_to},
          usr_rpl_pos_rel_to_{usr_rpl_pos_rel_to},
          usr_cmd_{init_cmd},
          use_aux2_{use_aux2},
          usr_rpl_aux2_pos_rel_to_{usr_rpl_aux2_pos_rel_to} {
      controller_->VerifySchemaVersion();
      std::cout << "CheckRegisterMapVersion passed." << std::endl;

      const auto& maybe_reply = controller_->SetStop();
      std::cout << (maybe_reply ? "Got" : "WARNING: Failed to get")
                << " reply from initial SetStop command for Servo ID " << id
                << "." << std::endl;

      SetBasePos(1.0);
      if (use_aux2) {
        SetBaseAux2Pos(1.0);
      }
    }

    moteus::Query::Result GetReply() { return usr_rpl(); }

    void SetReply(const moteus::Query::Result& new_sys_repl) {
      if (new_sys_repl.abs_position - sys_rpl_.abs_position > 0.5) {
        aux2_revs_--;
      } else if (new_sys_repl.abs_position - sys_rpl_.abs_position < -0.5) {
        aux2_revs_++;
      }
      sys_rpl_ = new_sys_repl;
    }

   protected:
    const bool InitSucceeded() {
      return use_aux2_
                 ? std::isfinite(base_pos_) && std::isfinite(base_aux2_pos_)
                 : std::isfinite(base_pos_);
    }

    const int GetId() { return controller_->options().id; }

    void SetCmdPosRelTo(CmdPosRelTo new_val) { usr_cmd_pos_rel_to_ = new_val; }

    void SetRplPosRelTo(RplPosRelTo new_val) { usr_rpl_pos_rel_to_ = new_val; }

    void SetRplAux2PosRelTo(RplPosRelTo new_val) {
      usr_rpl_aux2_pos_rel_to_ = new_val;
    }

    bool SetBasePos(const double time_limit) {
      return SetBase(time_limit, WhichPos::pINTERNAL);
    }

    bool SetBaseAux2Pos(const double time_limit) {
      return SetBase(time_limit, WhichPos::pAUX2);
    }

    bool SetBase(const double time_limit, WhichPos which_position) {
      const std::string which_str =
          (which_position == WhichPos::pINTERNAL) ? "" : "aux2 ";
      const int id = GetId();
      std::cout << "Attempting to get current " << which_str
                << "position from Servo ID " << id << "..." << std::endl;
      const double time_init = Utils::GetTime();
      while (true) {
        const auto& maybe_reply = controller_->SetQuery();
        if (!maybe_reply) {
          const double time_passed = Utils::GetTime() - time_init;
          const double time_left = time_limit - time_passed;
          if (time_left > 0.0) {
            std::cout << "Not getting reply from Servo ID " << id
                      << ", but will try " << time_left << " more seconds...\r";
            ::usleep(1.0 / 100.0 * 1e6);
            continue;
          } else {
            std::cout << "Failed to get reply from Servo ID " << id << " for "
                      << time_limit << " seconds.  " << "Base " << which_str
                      << "position setting failed." << std::endl;
            return false;
          }
        } else {
          const double cur_val = (which_position == WhichPos::pINTERNAL)
                                     ? maybe_reply->values.position
                                     : maybe_reply->values.abs_position;
          if (std::isfinite(cur_val)) {
            auto& base = (which_position == WhichPos::pINTERNAL)
                             ? base_pos_
                             : base_aux2_pos_;
            base = cur_val;
            std::cout << "Successfully set base " << which_str
                      << "position to current " << which_str
                      << "position for Servo ID " << id << ": " << base
                      << std::endl;
            SetReply(maybe_reply->values);
            return true;
          } else {
            const double time_passed = Utils::GetTime() - time_init;
            const double time_left = time_limit - time_passed;
            if (time_left > 0.0) {
              std::cout << "Current " << which_str << "position for Servo ID "
                        << id << " is NaN, but will try " << time_left
                        << " more seconds...\r";
              ::usleep(1.0 / 100.0 * 1e6);
              continue;
            } else {
              std::cout << "Failed to get finite value as the current "
                        << which_str << "position for Servo ID " << id
                        << " for " << time_limit << " seconds.  " << "Base "
                        << which_str << "position setting failed." << std::endl;
              return false;
            }
          }
        }
      }
    }

    const std::unique_ptr<moteus::Controller> controller_;
    CmdPosRelTo usr_cmd_pos_rel_to_;
    moteus::PositionMode::Command usr_cmd_;
    moteus::PositionMode::Command sys_cmd() {
      auto cmd = usr_cmd_;
      switch (usr_cmd_pos_rel_to_) {
        case CmdPosRelTo::cBASE:
          if (std::isnan(base_pos_)) {
            std::cout
                << "User requested a base position-relative command, "
                   "but base position is not set.  "
                   "User command position will be treated as absolute position."
                << std::endl;
          } else if (std::isnan(cmd.position)) {
            std::cout << "User requested a base position-relative command, "
                         "but command position is NaN.  NaN will be used for "
                         "system command position."
                      << std::endl;
          } else {
            cmd.position += base_pos_;
          }
          break;
        case CmdPosRelTo::cRECENT:
          if (std::isnan(cmd.position)) {
            std::cout << "User requested a recent position-relative command, "
                         "but command position is NaN.  NaN will be used for "
                         "system command position."
                      << std::endl;
          } else {
            if (updated_last_cycle_) {
              cmd.position += sys_rpl_.position;
            } else {
              cmd.position = NaN;
            }
          }
          break;
        default:
          break;
      }
      return cmd;
    }
    RplPosRelTo usr_rpl_pos_rel_to_;
    RplPosRelTo usr_rpl_aux2_pos_rel_to_;
    moteus::Query::Result sys_rpl_;  // aux2 position coiled.
                                     // Must update via dedicated setter
                                     // to track aux2 revolutions.
    moteus::Query::Result usr_rpl() {
      auto rpl = sys_rpl_;
      rpl.abs_position += aux2_revs_;  // Uncoil aux2 position.
      if (usr_rpl_pos_rel_to_ == RplPosRelTo::rBASE) {
        rpl.position -= base_pos_;
      }
      if (usr_rpl_aux2_pos_rel_to_ == RplPosRelTo::rBASE) {
        rpl.abs_position -= base_aux2_pos_;
      }
      return rpl;
    }
    bool updated_last_cycle_ = false;
    double base_pos_ = NaN;
    double base_aux2_pos_ = NaN;
    int aux2_revs_ = 0;
    bool use_aux2_;
  };

  virtual std::shared_ptr<Servo> InitServo(
      const int id, const int bus, const CmdPosRelTo cmd_pos_rel_to,
      const RplPosRelTo rpl_pos_rel_to,
      const moteus::PositionMode::Format& cmd_fmt,
      const moteus::PositionMode::Command& init_cmd,
      const moteus::Query::Format& rpl_fmt, const bool use_aux2,
      const RplPosRelTo rpl_aux2_pos_rel_to) {
    std::cout << "Initializing Servo ID " << id << " on bus " << bus << "..."
              << std::endl;
    const auto maybe_servo = std::make_shared<Servo>(
        id, bus, transport_, cmd_pos_rel_to, rpl_pos_rel_to, cmd_fmt, init_cmd,
        rpl_fmt, use_aux2, rpl_aux2_pos_rel_to);
    if (maybe_servo->InitSucceeded()) {
      return maybe_servo;
    } else {
      return nullptr;
    }
  }

  friend class LoopingThreadManager;
  class LoopingThreadManager {
    friend class ServoSystem;

   protected:
    LoopingThreadManager(
        ServoSystem* servo_system,
        std::function<void(ServoSystem*, std::atomic_bool*)> task,
        const std::vector<std::string>& aliases)
        : servo_system_{servo_system},
          task_{task},
          aliases_{aliases},
          thread_{nullptr} {
      if (aliases_.empty()) {
        std::cout << "LoopingThreadManager initialization failed since "
                     "no aliases are given for this thread."
                  << std::endl;
        return;
      }
      for (const auto& alias : aliases_) {
        servo_system_->threads_mgr_[alias] = this;
      }
    }

    void Start() {
      if (!thread_) {
        terminated_ = false;
        thread_ = new std::thread(task_, servo_system_, &terminated_);
        std::cout << aliases_[0]
                  << " thread successfully initialized and detached."
                  << std::endl;
      } else {
        std::cout << aliases_[0] << " thread is already running." << std::endl;
      }
    }

    void Terminate() {
      if (thread_) {
        terminated_ = true;
        std::cout << "Waiting for " << aliases_[0] << " thread to join..."
                  << std::endl;
        thread_->join();
        delete thread_;
        thread_ = nullptr;
        std::cout << aliases_[0] << " thread successfully terminated."
                  << std::endl;
      } else {
        std::cout << aliases_[0] << " thread has not been running."
                  << std::endl;
      }
    }

    bool IsRunning() { return !!thread_; }

   private:
    const std::vector<std::string> aliases_;
    ServoSystem* const servo_system_;
    std::thread* thread_;
    const std::function<void(ServoSystem*, std::atomic_bool*)> task_;
    std::atomic_bool terminated_;
  };

 public:
  ServoSystem(const std::map<int, int>& id_bus_map,
              const CmdPosRelTo cmd_pos_rel_to = CmdPosRelTo::cBASE,
              const RplPosRelTo rpl_pos_rel_to = RplPosRelTo::rBASE,
              const std::string& cmd_conf_dir = "../config",
              const std::string& rpl_conf_dir = "../config",
              const bool use_aux2 = false,
              const RplPosRelTo rpl_aux2_pos_rel_to = RplPosRelTo::rABSOLUTE)
      : exec_mgr_{this,
                  &ServoSystem::ExecuteCommand,
                  {"Executor", "exec", "x"}},
        ext_cmd_mgr_{this,
                     &ServoSystem::ExternalCommandGetter,
                     {"ExternalCommandGetter", "extcmd", "ec"}},
        ext_rpl_mgr_{this,
                     &ServoSystem::ExternalReplySender,
                     {"ExternalReplySender", "extrpl", "er"}} {
    /// Get default transport. Priority order: pi3hat > fdcanusb > socketcan
    transport_ = moteus::Controller::MakeSingletonTransport({});
    /* Print transport status and handle failure. */ {
      if (transport_) {
        std::string transport_name = Utils::GetClassName(transport_.get());
        if (!transport_name.empty()) {
          std::cout << "Default transport found: " << transport_name
                    << std::endl;
        } else {
          std::cout << "Default transport found, but failed to get its class "
                       "name."
                    << std::endl;
        }
      } else {
        std::cout << "Default transport not found.  Returning from "
                     "ServoSystem constructor without any initialization."
                  << std::endl;
        return;
      }
    }

    /// Parse the PositionMode config file.
    const auto cmd_conf = Parser::ParseCmdConf(cmd_conf_dir);
    const auto& cmd_fmt = cmd_conf.first;
    const auto& init_cmd = cmd_conf.second;

    /// Parse the Query config file.
    const auto rpl_fmt = Parser::ParseRplConf(rpl_conf_dir);
    if (use_aux2 && rpl_fmt.abs_position == moteus::Resolution::kIgnore) {
      std::cout << "WARNING: ServoSystem was told the user will use aux2, "
                   "but the reply resolution configuration for aux2 position "
                   "is set to kIgnore."
                << std::endl;
    } else if (!use_aux2 &&
               rpl_fmt.abs_position != moteus::Resolution::kIgnore) {
      std::cout << "WARNING: ServoSystem was told the user will not use aux2, "
                   "but the reply resolution configuration for aux2 position "
                   "is not set to kIgnore."
                << std::endl;
    }

    /// Initialize Servos.
    std::set<int> failed_ids;
    for (const auto& id_bus : id_bus_map) {
      const auto& id = id_bus.first;
      const auto& bus = id_bus.second;
      const auto& maybe_servo =
          InitServo(id, bus, cmd_pos_rel_to, rpl_pos_rel_to, cmd_fmt, init_cmd,
                    rpl_fmt, use_aux2, rpl_aux2_pos_rel_to);
      if (maybe_servo) {
        servos_.insert({id, maybe_servo});
        ids_.insert(id);
        std::cout << "Initialization succeeded for Servo ID " << id << "."
                  << std::endl;
      } else {
        failed_ids.insert(id);
        std::cout << "Initialization failed for Servo ID " << id << "."
                  << std::endl;
      }
    }

    /* Print Servo initialization result */ {
      if (!failed_ids.empty()) {
        std::cout << "Excluded Servos with the following IDs "
                     "from the Servo list: ";
        for (auto id : failed_ids) {
          std::cout << id << " ";
        }
        std::cout << std::endl;
      }

      if (!servos_.empty()) {
        std::cout << "Ready to control Servos of the following IDs: ";
        for (const auto& id_servo : servos_) {
          std::cout << id_servo.first << " ";
        }
        std::cout << std::endl;
      } else {
        std::cout << "No available Servos found." << std::endl;
      }
    }
  }

  ///////////////////////////////////////////////////////////////
  ///           Commands you can send to the Servos.          ///

  std::set<int> GetIds() { return ids_; }

  void ListenInternalCommand(bool listen) { listen_.internal = listen; }

  void ListenExternalCommand(bool listen) { listen_.external = listen; }

  void Command(const std::map<int, std::map<CmdItem, double>> cmds) {
    if (!listen_.internal) return;
    EmplaceCommand(cmds);
  }

  void CommandAll(const std::map<CmdItem, double> cmd) {
    if (!listen_.internal) return;
    std::map<int, std::map<CmdItem, double>> cmds;
    for (const auto id : ids_) {
      cmds[id] = cmd;
    }
    EmplaceCommand(cmds);
  }

  void GetReplyAll(char* const output, size_t size) {
    InternalReplySender(output, size);
  }

  void Fix(const std::set<int> ids) {
    std::map<int, std::map<CmdItem, double>> cmds;
    for (const auto id : ids) {
      const auto& servo = Utils::SafeAt(servos_, id);
      if (servo) {
        cmds[id][CmdItems::position] = NaN;
        cmds[id][CmdItems::velocity] = 0.0;
        cmds[id][CmdItems::feedforward_torque] = 0.0;
      }
    }
    EmplaceCommand(cmds);
  }

  void FixAll() { Fix(ids_); }

  void Stop(const std::set<int> ids) {
    for (const auto id : ids) {
      const auto& servo = Utils::SafeAt(servos_, id);
      if (servo) {
        servo.value()->controller_->SetStop();
      }
    }
  }

  void StopAll() { Stop(ids_); }

  void SetBasePosition(const std::set<int> ids) {
    for (const auto id : ids) {
      const auto& servo = Utils::SafeAt(servos_, id);
      if (servo) {
        if (!servo.value()->SetBasePos(0.0)) {
          std::cout << "Base position setting failed for ID " << id << "."
                    << std::endl;
        } else {
          std::cout << "Base position setting succeeded for ID " << id << "."
                    << std::endl;
        }
      } else {
        std::cout << "No Servo of ID " << id
                  << " found.  Base position setting failed." << std::endl;
      }
    }
  }

  void SetBasePositionAll() { SetBasePosition(ids_); }

  void SetBaseAux2Position(const std::set<int> ids) {
    for (const auto id : ids) {
      const auto& servo = Utils::SafeAt(servos_, id);
      if (servo) {
        if (!servo.value()->SetBaseAux2Pos(0.0)) {
          std::cout << "Base aux2 position setting failed for ID " << id << "."
                    << std::endl;
        } else {
          std::cout << "Base aux2 position setting succeeded for ID " << id
                    << "." << std::endl;
        }
      } else {
        std::cout << "No Servo of ID " << id
                  << " found.  Base aux2 position setting failed." << std::endl;
      }
    }
  }

  void SetBaseAux2PositionAll() { SetBaseAux2Position(ids_); }

  void SetCmdPosRelTo(const std::set<int> ids, CmdPosRelTo new_val) {
    for (const auto id : ids) {
      const auto& servo = Utils::SafeAt(servos_, id);
      if (servo) {
        servo.value()->SetCmdPosRelTo(new_val);
      }
    }
  }

  void SetCmdPosRelToAll(CmdPosRelTo new_val) { SetCmdPosRelTo(ids_, new_val); }

  void SetRplPosRelTo(const std::set<int> ids, RplPosRelTo new_val) {
    for (const auto id : ids) {
      const auto& servo = Utils::SafeAt(servos_, id);
      if (servo) {
        servo.value()->SetRplPosRelTo(new_val);
      }
    }
  }

  void SetRplPosRelToAll(RplPosRelTo new_val) { SetRplPosRelTo(ids_, new_val); }

  void SetRplAux2PosRelTo(const std::set<int> ids, RplPosRelTo new_val) {
    for (const auto id : ids) {
      const auto& servo = Utils::SafeAt(servos_, id);
      if (servo) {
        servo.value()->SetRplAux2PosRelTo(new_val);
      }
    }
  }

  void SetRplAux2PosRelToAll(RplPosRelTo new_val) {
    SetRplAux2PosRelTo(ids_, new_val);
  }

  void StartThread(const std::string& alias) {
    const auto& maybe_thread_manager = Utils::SafeAt(threads_mgr_, alias);
    if (maybe_thread_manager) {
      maybe_thread_manager.value()->Start();
    } else {
      std::cout << "Cannot find thread with alias " << alias << "."
                << std::endl;
    }
  }

  void StartThreadAll() {
    std::set<LoopingThreadManager*> threads;
    for (const auto& pair : threads_mgr_) {
      threads.insert(pair.second);
    }
    for (const auto& thread : threads) {
      StartThread(thread);
    }
  }

  void TerminateThread(const std::string& alias) {
    const auto& maybe_thread_manager = Utils::SafeAt(threads_mgr_, alias);
    if (maybe_thread_manager) {
      maybe_thread_manager.value()->Terminate();
    } else {
      std::cout << "Cannot find thread with alias " << alias << "."
                << std::endl;
    }
  }

  void TerminateThreadAll() {
    std::set<LoopingThreadManager*> threads;
    for (const auto& pair : threads_mgr_) {
      threads.insert(pair.second);
    }
    for (const auto& thread : threads) {
      TerminateThread(thread);
    }
  }

  void PrintThreadAll() {
    std::set<LoopingThreadManager*> threads;
    for (const auto& pair : threads_mgr_) {
      threads.insert(pair.second);
    }
    int i = 1;
    for (const auto& thread : threads) {
      std::cout << "Thread #" << i++ << ":" << std::endl;
      std::cout << "    Aliases: ";
      for (const auto& alias : thread->aliases_) {
        std::cout << alias << " || ";
      }
      std::cout << std::endl
                << "    Is Running?: "
                << (thread->IsRunning() ? "true" : "false") << std::endl;
    }
  }

  ///           Commands you can send to the Servos.          ///
  ///////////////////////////////////////////////////////////////

 protected:
  std::set<int> ids_;
  std::map<int, std::shared_ptr<Servo>> servos_;  // ID -> Servo
  std::shared_ptr<moteus::Transport> transport_;
  struct {
    bool internal = true;
    bool external = true;
  } listen_;
  const unsigned int cycle_period_us_ =
      static_cast<unsigned int>(1.0 / 100.0 * 1e6);
  std::mutex servo_access_mutex_;
  std::map<std::string, LoopingThreadManager*> threads_mgr_;
  LoopingThreadManager exec_mgr_;
  LoopingThreadManager ext_cmd_mgr_;
  LoopingThreadManager ext_rpl_mgr_;

  virtual void InternalReplySender(char* const output, size_t size) {
    char* current = output;
    int written;
    for (const auto& id_servo : servos_) {
      const auto id = id_servo.first;
      const auto& servo = id_servo.second;
      moteus::Query::Result usr_rpl;
      bool updated_this_cycle;

      {
        std::lock_guard<std::mutex> lock(servo_access_mutex_);
        usr_rpl = servo->usr_rpl();
        updated_this_cycle = servo->updated_last_cycle_;
      }

      if (updated_this_cycle) {
        written =
            ::snprintf(current, size,
                       "id=%2d | mode=%2d | "
                       "pos=%3.3f | aux2_pos=%3.3f | vel=%3.3f | trq=%3.3f | "
                       "q=%3.3f | d=%3.3f | volt=%3.2f | power=%3.3f | "
                       "motor_temp=%3.3f | temp=%2.1f | "
                       "trj=%s | home=%2d | "
                       "fault=%2d\n",
                       id, static_cast<int>(usr_rpl.mode),  //
                       usr_rpl.position, usr_rpl.abs_position, usr_rpl.velocity,
                       usr_rpl.torque,  //
                       usr_rpl.q_current, usr_rpl.d_current, usr_rpl.voltage,
                       usr_rpl.power,                                   //
                       usr_rpl.motor_temperature, usr_rpl.temperature,  //
                       usr_rpl.trajectory_complete ? "true" : "false",
                       static_cast<int>(usr_rpl.home_state),  //
                       usr_rpl.fault);
      } else {
        written =
            ::snprintf(current, size, "id=%2d | NOT UPDATED THIS CYCLE\n", id);
      }

      if (written < 0) {
        std::cout << "Error occured during printing to designated pointer."
                  << std::endl;
        return;
      } else if (written >= size) {
        std::cout << "Insufficient size to accommodate replies." << std::endl;
        return;
      }

      current += written;
      size -= written;
    }
  }

  virtual void ExternalReplySender(std::atomic_bool* terminated) {
    std::cout << "ExternalReplySender thread is running..." << std::endl;

    /// Probably let a ROS2 publisher publish Servo replies
    /// or send encoded Servo replies as UDP packets.
  }

  virtual void ExternalCommandGetter(std::atomic_bool* terminated) {
    std::cout << "ExternalCommandGetter thread is running..." << std::endl;

    while (!((*terminated).load())) {
      std::string input;
      std::getline(std::cin, input);
      if (!listen_.external || input.empty()) continue;
      EmplaceCommand(Parser::ParseStrInput(input));
    }
  }

  void EmplaceCommand(const std::map<int, std::map<CmdItem, double>>& cmds) {
    std::cout << "Parsing command..." << std::endl;
    for (const auto& id_cmd : cmds) {
      const auto id = id_cmd.first;
      const auto& cmd = id_cmd.second;
      const auto& maybe_servo = Utils::SafeAt(servos_, id);
      if (!maybe_servo) {
        std::cout << "No Servo of ID " << id
                  << " found on the Servo list.  Ignoring command for this ID."
                  << std::endl;
        continue;
      }
      const auto& servo = maybe_servo.value();

      std::cout << "Parsing command for Servo ID " << id << "..." << std::endl;
      for (const auto& item_val : cmd) {
        const auto item = item_val.first;
        auto val = item_val.second;
        std::cout << "Parsing command item " << item.GetName() << "..."
                  << std::endl;
        {
          std::lock_guard<std::mutex> lock(servo_access_mutex_);
          if (*CmdItems::Get(item,
                             servo->controller_->options().position_format) ==
              moteus::Resolution::kIgnore) {
            std::cout << "ServoSystem command resolution is set to "
                      << item.GetName() << " = "
                      << "kIgnore.  Ignoring command." << std::endl;
            continue;
          }
          *CmdItems::Get(item, servo->usr_cmd_) = val;
        }
        std::cout << "Successfully set command " << item.GetName() << " = "
                  << val << " for Servo ID " << id << "." << std::endl;
      }
    }
  }

 private:
  void ExecuteCommand(std::atomic_bool* terminated) {
    std::cout << "Executor thread is running..." << std::endl;

    if (servos_.empty()) {
      std::cout << "No Servos found.  Executor thread terminating."
                << std::endl;
      return;
    }

    while (!((*terminated).load())) {
      ::usleep(cycle_period_us_);

      std::cout << "Command execution begins." << std::endl;
      std::vector<moteus::CanFdFrame> cmd_frames;
      {
        std::lock_guard<std::mutex> lock(servo_access_mutex_);
        for (auto& id_servo : servos_) {
          auto id = id_servo.first;
          auto& servo = id_servo.second;
          const auto sys_cmd = servo->sys_cmd();
          std::cout << "Making CAN FD frame for Servo ID " << id << ".  "
                    << "System command position: " << servo->sys_cmd().position
                    << std::endl;
          cmd_frames.push_back(servo->controller_->MakePosition(sys_cmd));
          servo->updated_last_cycle_ = false;
        }

        std::vector<moteus::CanFdFrame> reply_frames;
        transport_->BlockingCycle(&cmd_frames[0], cmd_frames.size(),
                                  &reply_frames);

        for (const auto& frame : reply_frames) {
          int id = frame.source;
          const auto& maybe_servo = Utils::SafeAt(servos_, id);
          if (maybe_servo) {
            auto& servo = maybe_servo.value();
            servo->SetReply(moteus::Query::Parse(frame.data, frame.size));
            servo->updated_last_cycle_ = true;
          }
        }
      }
      std::cout << "Command execution complete." << std::endl;
    }
  }

  void StartThread(LoopingThreadManager* thread) {
    if (thread) thread->Start();
  }

  void TerminateThread(LoopingThreadManager* thread) {
    if (thread) thread->Terminate();
  }
};

}  // namespace som

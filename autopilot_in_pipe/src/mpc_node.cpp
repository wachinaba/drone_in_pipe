#include <ros/ros.h>

#include <ct/optcon/optcon.h>
#include <autopilot_in_pipe/AdrienSystem.h>

class MPCNode
{
public:
  MPCNode(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
  {
    system_ = std::make_shared<ct::core::AdrienSystem<double>>();
    linearizer_ = std::make_shared<ct::optcon::SystemLinearizer<kStateDim, kControlDim>>(system_);

    intermediate_cost_ = std::make_shared<ct::optcon::TermQuadratic<kStateDim, kControlDim>>();
    final_cost_ = std::make_shared<ct::optcon::TermQuadratic<kStateDim, kControlDim>>();

    intermidiate_cost_->loadConfigFile(pnh_.getParam<std::string>("intermediate_cost_file"));
    final_cost_->loadConfigFile(pnh_.getParam<std::string>("final_cost_file"));

    cost_function_ = std::make_shared<ct::optcon::CostFunctionQuadratic<kStateDim, kControlDim>>();
    cost_function_->addIntermediateTerm(intermidiate_cost_);
    cost_function_->addFinalTerm(final_cost_);

    opt_problem_ = std::make_shared<ct::optcon::ContinuousOptConProblem<kStateDim, kControlDim>>(
        0.0, 1.0, horizon_, system_, cost_function_);

    ilqr_settings_ = std::make_shared<ct::optcon::NLOptConSettings>();
    ilqr_settings_->dt = 0.01;
    ilqr_settings_->integrator = ct::core::IntegrationType::EULERCT;
    ilqr_settings_->discretization = ct::optcon::NLOptConSettings::APPROXIMATION::FORWARD_EULER;
    ilqr_settings_->max_iterations = 1;
    ilqr_settings_->nlocp_algorithm = ct::optcon::NLOptConSettings::NLOCP_ALGORITHM::ILQR;
    ilqr_settings_->lqocp_solver = ct::optcon::NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;

    mpc_settings_ = std::make_shared<ct::optcon::mpc_settings>();
    mpc_settings_->stateForwardIntegration_ = true;
    mpc_settings_->postTruncation_ = true;
    mpc_settings_->measureDelay_ = true;
    mpc_settings_->delayMeasurementMultiplier_ = 1.0;
    mpc_settings_->mpc_mode = ct::optcon::MPC_MODE::FIXED_FINAL_TIME;
    mpc_settings_->coldStart_ = false;

    mpc_ =
        ct::optcon::MPC<ct::optcon::NLOptConSolver<kStateDim, kControlDim>>(opt_problem, ilqr_settings_, mpc_settings_);

    solver_ = std::make_shared<ct::optcon::NLOptConSolver<kStateDim, kControlDim>>(opt_problem_, *settings_);
  }
  ~MPCNode();

  void run();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  const size_t kStateDim = 13;   // position, quaternion, velocity, angular velocity
  const size_t kControlDim = 4;  // thrust

  std::shared_ptr<ct::core::AdrienSystem<double>> system_;

  std::shared_ptr<ct::optcon::SystemLinearizer<kStateDim, kControlDim>> linearizer_;

  std::shared_ptr<ct::optcon::TermQuadratic<kStateDim, kControlDim>> intermediate_cost_;
  std::shared_ptr<ct::optcon::TermQuadratic<kStateDim, kControlDim>> final_cost_;

  std::shared_ptr<ct::optcon::CostFunctionQuadratic<kStateDim, kControlDim>> cost_function_;

  std::shared_ptr<ct::optcon::ContinuousOptConProblem<kStateDim, kControlDim>> opt_problem_;

  std::shared_ptr<ct::optcon::NLOptConSettings> ilqr_settings_;

  std::shared_ptr<ct::optcon::NLOptConSolver<kStateDim, kControlDim>> solver_;

  std::shared_ptr<ct::optcon::mpc_settings> mpc_settings_;

  ct::optcon::MPC<ct::optcon::NLOptConSolver<kStateDim, kControlDim>> mpc_;

  size_t horizon_;
}

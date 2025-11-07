// src/current_system.cc
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Element.hh>
#include <gz/transport/Node.hh>
#include <string>
#include <vector>
#include <algorithm>
#include <optional>

using namespace gz;

class CurrentSystem :
    public sim::System,
    public sim::ISystemConfigure,
    public sim::ISystemPreUpdate
{
public:
  void Configure(const sim::Entity & /*entity*/,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 sim::EntityComponentManager & /*ecm*/,
                 sim::EventManager & /*eventMgr*/) override
  {
    // Params
    this->velCurrent = math::Vector3d(
      sdf->Get<double>("vx", 0.3).first,
      sdf->Get<double>("vy", 0.0).first,
      sdf->Get<double>("vz", 0.0).first);

    this->rho = sdf->Get<double>("rho", 1000.0).first;
    this->Cd  = sdf->Get<double>("drag_coeff", 0.8).first;
    this->A   = sdf->Get<double>("area", 0.03).first;
    this->node.Subscribe("/current/set",
                     &CurrentSystem::CurrentCmd,
                     this);

    // Optional list of affected models
    if (sdf->HasElement("affected_models"))
    {
      auto am = sdf->FindElement("affected_models");
      if (am)
      {
        auto child = am->GetFirstElement();
        while (child)
        {
          if (child->GetName() == "model")
            this->modelNames.push_back(child->Get<std::string>());
          child = child->GetNextElement();
        }
      }
    }
    // Don't try to collect links here; models may not exist yet.
    this->needsDiscovery = true;
  }
    gz::transport::Node node;

    void CurrentCmd(const gz::msgs::Vector3d &_msg)
    {
    this->velCurrent = { _msg.x(), _msg.y(), _msg.z() };
    std::cerr << "[CurrentSystem] Updated current to: "
                << this->velCurrent << "\n";
    }
  void PreUpdate(const sim::UpdateInfo &info,
                 sim::EntityComponentManager &ecm) override
  {
    if (info.paused) return;

    // Discover links once models are present
    if (this->needsDiscovery)
    {
      this->links.clear();

      ecm.Each<sim::components::Link, sim::components::ParentEntity>(
        [&](const sim::Entity &linkEnt,
            const sim::components::Link*,
            const sim::components::ParentEntity* parent) -> bool
        {
          bool include = this->modelNames.empty();
          if (!include)
          {
            auto name = ecm.Component<sim::components::Name>(parent->Data());
            if (name)
            {
              // exact match; if your model is named bluerov2_0, add that name here
              include = std::find(this->modelNames.begin(), this->modelNames.end(),
                                  name->Data()) != this->modelNames.end();
            }
          }
          if (include)
            this->links.push_back(linkEnt);
          return true;
        });

      // If nothing found, keep trying next tick (maybe models spawn later)
      this->needsDiscovery = this->links.empty();

      // Optional: brief log to confirm
      if (this->links.empty())
      {
        // std::cerr is fine if you don't want to pull in gz::common console
        std::cerr << "[CurrentSystem] No links found yet; will retry.\n";
      }
      else
      {
        std::cerr << "[CurrentSystem] Will apply current to " << this->links.size()
                  << " link(s).\n";
      }
    }

    if (this->links.empty())
      return; // nothing to do yet

    const math::Vector3d vCurr = this->velCurrent;
    const double rho_ = this->rho;
    const double Cd_  = this->Cd;
    const double A_   = this->A;

    for (const auto &linkEnt : this->links)
    {
      sim::Link link(linkEnt);
      auto vLinkOpt = link.WorldLinearVelocity(ecm);
      if (!vLinkOpt) continue;

      math::Vector3d vRel = vLinkOpt.value() - vCurr;
      const double speed = vRel.Length();
      if (speed < 1e-6) continue;

      math::Vector3d force = -0.5 * rho_ * Cd_ * A_ * speed * vRel;
      link.AddWorldWrench(ecm, force, math::Vector3d::Zero);
    }
  }

private:
  math::Vector3d velCurrent{0.3, 0.0, 0.0}; // m/s ENU (+X east, +Y north, +Z up)
  double rho{1000.0};
  double Cd{0.8};
  double A{0.03};

  std::vector<std::string> modelNames;
  std::vector<sim::Entity> links;
  bool needsDiscovery{true};
};

GZ_ADD_PLUGIN(CurrentSystem,
              sim::System,
              CurrentSystem::ISystemConfigure,
              CurrentSystem::ISystemPreUpdate);

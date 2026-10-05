// Offline SDFormat inspection only; no ROS or physics system is initialized.
#include <sdf/sdf.hh>
#include <iostream>
#include <stdexcept>
int main(int argc, char **argv)
{
  try {
    for (int i = 1; i < argc; ++i) {
      sdf::Root root; const auto errors = root.Load(argv[i]);
      if (!errors.empty()) { for (const auto &e : errors) std::cerr << e << '\n'; return 1; }
      const auto *model = root.WorldByIndex(0)->ModelByName("rm_sentry_2027");
      if (!model) throw std::runtime_error("missing original robot");
      for (const auto &name : {"front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"}) {
        auto element = model->LinkByName(name)->CollisionByIndex(0)->Element()
          ->GetElement("surface")->GetElement("friction")->GetElement("ode")->GetElement("fdir1");
        std::cout << argv[i] << '\t' << name << '\t' << element->HasAttribute("ignition:expressed_in")
          << '\t' << element->HasAttribute("ns0:expressed_in") << '\t' << element->Get<ignition::math::Vector3d>() << '\n';
      }
    }
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}

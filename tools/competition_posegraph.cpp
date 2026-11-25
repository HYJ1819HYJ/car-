#include <karto_sdk/Mapper.h>

#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  karto::Mapper mapper;
  mapper.LoadFromFile(argv[1]);
  std::cout << std::setprecision(16);
  for (const auto* scan : mapper.GetAllProcessedScans()) {
    const auto& pose = scan->GetCorrectedPose();
    std::cout << scan->GetUniqueId() << ',' << scan->GetTime() << ','
              << pose.GetX() << ',' << pose.GetY() << '\n';
  }
}

#include "pr_utils/pr_ptu.h"
#include "pr_utils/pr_pitcam.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

int main(int argc, char** argv) {
  if(argc != 4 and argc != 5) {
    std::cout << "usage: ./pitcam_util <pan> <tilt> <exposure> [<filename>.jpg]" << std::endl;
    return 0;
  }

  int pan  = std::atoi(argv[1]);
  int tilt = std::atoi(argv[2]);
  int exposure = std::atoi(argv[3]);

  pr::PanTiltController ptu;
  pr::PitCamera cam;

  ptu.set_pan_deg(pan);
  ptu.set_tilt_deg(tilt);

  if( exposure == 0 ) {
    exposure = cam.get_autoexposure();
  }

  auto image = cam.capture(exposure);
  std::cout << "Pan:  " << pan << '\n';
  std::cout << "Tilt: " << tilt << '\n';
  std::cout << "Exposure: " << cam.get_exposure() << '\n';

  if( argc == 5 ) {
    stbi_write_jpg(argv[4], image.cols, image.rows, 3, image.data.data(), 90);
  }


  return 0;
}

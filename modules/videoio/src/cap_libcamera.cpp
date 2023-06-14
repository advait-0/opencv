#include <iomanip>
#include <iostream>
#include <memory>

#include <libcamera/libcamera.h>

#define TIMEOUT_SEC 3

using namespace libcamera;
static std::shared_ptr<Camera> camera;

std::string cameraName(Camera* camera)
{
    const ControlList& props = camera->properties();
    std::string name;

    const auto& location = props.get(properties::Location);
    if (location) {
        switch (*location) {
            case properties::CameraLocationFront:
                name = "Internal front camera";
                break;
            case properties::CameraLocationBack:
                name = "Internal back camera";
                break;
            case properties::CameraLocationExternal:
                name = "External camera";
                const auto& model = props.get(properties::Model);
                if (model)
                    name += " '" + *model + "'";
                break;
        }
    }

    name += " (" + camera->id() + ")";

    return name;
}

int main()
{
    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();

    for (const auto& camera : cm->cameras())
        std::cout << " - " << cameraName(camera.get()) << std::endl;

    if (cm->cameras().empty()) {
        std::cout << "No cameras were identified on the system" << std::endl;
        cm->stop();
        return EXIT_FAILURE;
    }

    std::string cameraId = cm->cameras()[0]->id();
    camera = cm->get(cameraId);
    std::cout << "Cameras available yayy" << cameraName(camera.get()) << std::endl;

    return 0;
}


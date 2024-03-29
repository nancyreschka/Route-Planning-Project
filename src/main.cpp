#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;    
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // get user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below.
    float start_x = 0.0f;
    float start_y = 0.0f;
    float end_x = 0.0f;
    float end_y = 0.0f;

    // get user input for the start and end node
    std::cout << " Please enter the following values:\n";
    std::cout << " (values should be in the Range of 0 to 100, otherwise they will be set to max or min:\n";
    std::cout << " start x = ";
    std::cin >> start_x;
    while (!std::cin || start_x < 0 || start_x > 100)
    {
        std::cout << "Invalid value. Enter a start x form 0 to 100: ";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> start_x;
    }
    std::cout << " start y = ";
    std::cin >> start_y;
    while (!std::cin || start_y < 0 || start_y > 100)
    {
        std::cout << "Invalid value. Enter a start y form 0 to 100: ";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> start_y;
    }
    std::cout << " end x = ";
    std::cin >> end_x;
    while (!std::cin || end_x < 0 || end_x > 100)
    {
        std::cout << "Invalid value. Enter a end x form 0 to 100: ";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> end_x;
    }
    std::cout << " end y = ";
    std::cin >> end_y;
    while (!std::cin || end_y < 0 || end_y > 100)
    {
        std::cout << "Invalid value. Enter a end y form 0 to 100: ";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> end_y;
    }
    std::cout << " \n ";

    std::cout << "Start x " << start_x << " y " << start_y << " - End x " << end_x << " y " << end_y << "\n";

    // Build Model.
    RouteModel model{osm_data};
    
    // Perform search and render results.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();
    std::cout << "Distance of the found path is: " << route_planner.GetDistance() << "m \n";
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}

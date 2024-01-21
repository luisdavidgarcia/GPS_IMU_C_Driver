#include "../../include/gps.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <curl/curl.h>
#include <unistd.h>
#include <filesystem>

// Function to write data received from HTTP request
static size_t WriteCallback(void *contents, size_t size, size_t nmemb, std::string *s) {
    size_t newLength = size * nmemb;
    try {
        s->append((char*)contents, newLength);
    } catch(std::bad_alloc &e) {
        // handle memory problem
        return 0;
    }
    return newLength;
}

// Function to download map
void download_map(const std::vector<std::pair<double, double> >& coordinates, const std::string& api_key = "", int zoom = 18, const std::string& size = "800x600", const std::string& filename = "map_image.png") {
    CURL *curl;
    CURLcode res;
    std::string readBuffer;

    curl = curl_easy_init();
    if(curl) {
        std::string base_url = "https://maps.googleapis.com/maps/api/staticmap?";
        std::string markers;
        for (size_t i = 0; i < coordinates.size(); ++i) {
            if (i != 0) markers += "|";
            markers += "color:red|" + std::to_string(coordinates[i].first) + "," + std::to_string(coordinates[i].second);
        }
        std::string url = base_url + "zoom=" + std::to_string(zoom) + "&size=" + size + "&markers=" + markers + "&key=" + api_key;
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        std::ofstream outfile(filename, std::ofstream::binary);
        outfile << readBuffer;
    }
}

int main() {
    Gps gps_module;
   // Use getenv to retrieve the API key from the environment variables
    const char* env_api_key = std::getenv("API_KEY");

    // Convert the C-style string to a std::string
    std::string api_key = env_api_key ? std::string(env_api_key) : "";

    // Check if the API key is empty and handle the case where it is not found
    if (api_key.empty()) {
        std::cerr << "API_KEY environment variable is not set." << std::endl;
        // Handle the error appropriately
        return 1; // Return a non-zero value to indicate failure
    }

    std::vector<std::pair<double, double> >  all_coordinates;

    while(1) {
        PVTData data = gps_module.GetPvt(true, 1);
        if (data.year == 2023) {
            std::pair<double, double> coordinates = std::make_pair(data.latitude, data.longitude);
            all_coordinates.push_back(coordinates);
            printf("Length of all_coordinates: %d\n", all_coordinates.size());
            printf("Coordinates: %f, %f\n", data.latitude, data.longitude);
           download_map(all_coordinates, api_key);
           system("xdg-open map_image.png"); // Open the image using the default application
           system("pkill -f /usr/bin/eom");
           system("rm map_image.png");
        }

        sleep(1);
    }

    return 0;
}

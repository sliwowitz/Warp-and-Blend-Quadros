#include "stdafx.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <streambuf>
#include <windows.h>

#include <regex>

#include <Eigen\Geometry>

#include "nvapi.h"

#include <string>
#include <regex>

#include <map>

#include <utility>
#include <iostream>

#include "lodepng.h"

using namespace std;
using namespace Eigen;

constexpr static int NUM_VERTICES = 4;

struct RectCoords {
    Vector2f tl;
    Vector2f bl;
    Vector2f tr;
    Vector2f br;

};

vector<float> get_warping_vertices(float srcLeft, float srcTop, 
    float srcWidth, float srcHeight, RectCoords target, int rotate = 0) {

    rotate = 2;
    RectCoords tgt;
    switch (rotate) {
    case 1:
        tgt.tl = target.bl;
        tgt.bl = target.br;
        tgt.tr = target.tl;
        tgt.br = target.tr;
        break;
    case 2:
        tgt.tl = target.tl;
        tgt.bl = target.tr;
        tgt.tr = target.bl;
        tgt.br = target.br;
        break;
    default: 
        tgt = target;
    }

    // find intersection between diagonals
    Hyperplane<float,2> tl_br = Hyperplane<float,2>::Through(tgt.tl,tgt.br);
    Hyperplane<float,2> tr_bl = Hyperplane<float,2>::Through(tgt.tr,tgt.bl);
    Vector2f intersection = tl_br.intersection(tr_bl);

    // quadrilateral projective interpolation
    // FIXME: this isn't working, non-continuous seams
    // see: http://www.reedbeta.com/blog/2012/05/26/quadrilateral-interpolation-part-1/

    // calculate distances from vertices to intersection
    float d_tl = (tgt.tl-intersection).norm();
    float d_tr = (tgt.tr-intersection).norm();
    float d_bl = (tgt.bl-intersection).norm();
    float d_br = (tgt.br-intersection).norm();

    // fourth texture coordinate 'q' plays the role of '1/z' in texturing
    // FIXME - this projective interpolation does not work across seams
    // FIXME - we need some sort of bilateral interpolation
    float q_tl = (d_tl + d_br) / (d_br);
    float q_tr = (d_tr + d_bl) / (d_bl);
    float q_bl = (d_bl + d_tr) / (d_tr);
    float q_br = (d_br + d_tl) / (d_tl);

    // XYUVRW coordinates to return
    //  (0)  ----------------- (2)
    //       |             / |
    //       |           /   |
    //       |         /     |
    //       |       /       |
    //       |     /         |
    //       |   /           |
    //       | /             |
    //   (1) ----------------- (3)

    vector<float> tl{ tgt.tl.x(), tgt.tl.y(), srcLeft * q_tl / 2, srcTop * q_tl / 2, 0.0f, q_tl /2};
    vector<float> bl{ tgt.bl.x(), tgt.bl.y(), srcLeft * q_bl / 2, (srcTop + srcHeight) * q_bl / 2, 0.0f, q_bl /2};
    vector<float> tr{ tgt.tr.x(), tgt.tr.y(), (srcWidth + srcLeft) * q_tr / 2, srcTop * q_tr / 2,0.0f, q_tr /2};
    vector<float> br{ tgt.br.x(), tgt.br.y(), (srcWidth + srcLeft) * q_br / 2, (srcTop + srcHeight) * q_br / 2, 0.0f, q_br /2};

    vector<float> coords_vector{};
    
    coords_vector.insert(end(coords_vector), begin(tl), end(tl));
    coords_vector.insert(end(coords_vector), begin(bl), end(bl));
    coords_vector.insert(end(coords_vector), begin(tr), end(tr));
    coords_vector.insert(end(coords_vector), begin(br), end(br));

    return coords_vector;
}

RectCoords read_warping_vertices(int desktop, int row, int col) {

    // read coordinates file into str
    std::ifstream f("coords_for_warp.txt");
    if (!f.is_open()) {
        std::cout << "Failed to open coords file\n";
        throw "Failed to open coords file";
    }
    std::string str((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    printf("Using warping:\n----------------\n%s\n----------------\n", str.c_str());
    // extract float coordinates from text using regex
    std::regex regex("DESKTOP: " + to_string(desktop) + " ROW: " + to_string(row) + " COL: " + to_string(col) + "\\n"
            "\\(\\s*(\\-?[0-9]+.[0-9]+),\\s*(\\-?[0-9]+.[0-9]+)\\)\\s+"     // top left corner (x,y)
            "\\(\\s*(\\-?[0-9]+.[0-9]+),\\s*(\\-?[0-9]+.[0-9]+)\\)\\s*\\n"// top right corner (x,y)
            "\\(\\s*(\\-?[0-9]+.[0-9]+),\\s*(\\-?[0-9]+.[0-9]+)\\)\\s+"// bottom left corner (x,y)
            "\\(\\s*(\\-?[0-9]+.[0-9]+),\\s*(\\-?[0-9]+.[0-9]+)\\)"// bottom right corner (x,y)
    );
    std::smatch match;
    std::regex_search(str, match, regex);

    std::cout << "coords found:" << match.size() << std::endl;

    // extract coordinates of quadrilateral corners
    Vector2f tl = Vector2f(stof(match[1].str()), stof(match[2].str()));
    Vector2f tr = Vector2f(stof(match[3].str()), stof(match[4].str()));
    Vector2f bl = Vector2f(stof(match[5].str()), stof(match[6].str()));
    Vector2f br = Vector2f(stof(match[7].str()), stof(match[8].str()));

    return RectCoords { tl, tr, bl, br };
}

void warp_display(NV_SCANOUT_INFORMATION &scanInfo, NvU32 display_id, vector<float> &vertices,
        NV_SCANOUT_WARPING_DATA &warpingData) {
    printf("vertices: %6.0f, %6.0f, %6.0f, %6.0f, %6.3f, %6.3f\n", vertices[0], vertices[1], vertices[2], vertices[3],
            vertices[4], vertices[5]);
    printf("vertices: %6.0f, %6.0f, %6.0f, %6.0f, %6.3f, %6.3f\n", vertices[6], vertices[7], vertices[8], vertices[9],
            vertices[10], vertices[11]);
    printf("vertices: %6.0f, %6.0f, %6.0f, %6.0f, %6.3f, %6.3f\n", vertices[12], vertices[13], vertices[14],
            vertices[15], vertices[16], vertices[17]);
    printf("vertices: %6.0f, %6.0f, %6.0f, %6.0f, %6.3f, %6.3f\n", vertices[18], vertices[19], vertices[20],
            vertices[21], vertices[22], vertices[23]);

    printf("Warping\n");
    warpingData.version = NV_SCANOUT_WARPING_VER;
    warpingData.numVertices = NUM_VERTICES;
    warpingData.vertexFormat = NV_GPU_WARPING_VERTICE_FORMAT_TRIANGLESTRIP_XYUVRQ;
    warpingData.textureRect = &scanInfo.sourceDesktopRect;
    warpingData.vertices = vertices.data();

    // This call does the Warp
    int maxNumVertices = 0;
    int sticky = 0;
    auto error = NvAPI_GPU_SetScanoutWarping(display_id, &warpingData, &maxNumVertices, &sticky);
    if (error != NVAPI_OK) {
        char estring[1024];
        NvAPI_GetErrorMessage(error, estring);
        printf("NvAPI_GPU_SetScanoutWarping: %s\n", estring);
    }
    if (maxNumVertices != NUM_VERTICES) {
        std::cout << "maxNumVertices != NUM_VERTICES" << std::endl;
    }
    cout << "Will persist on reboot? " << (sticky ? "yes" : "no") << endl;
}

//TODO: interactivity possible with windows conio.h/getch(), or (better) ncurses
void move_vertex(Vector2f * vertex) {
    char c = ' ';
    float invalid = 0.0f;
    while (c != 'q') {
        cout << "Move along axis (x/y = AXIS, q = EXIT): ";
        cin>> c;

        switch (c) {
        case 'x':
            break;
        case 'y':
            break;
        default:
            continue;
        }

        float offset = 0;
        cout << "Offset: ";
        cin >> offset;

        switch (c) {
        case 'x':
            vertex->x() += offset;
            break;
        case 'y':
            vertex->y() += offset;
            break;
        default:
            continue;
        }
    }
}

int main(int argc, char **argv) {
    bool interactive = false;
    if (argc > 1) {
        interactive = true;
    }

    NvAPI_Status error;
    NvPhysicalGpuHandle nvGPUHandles[NVAPI_MAX_PHYSICAL_GPUS];
    NvU32 gpuCount = 0;
    NvU32 gpu;
    NvU32 outputMask = 0;

    NV_SCANOUT_WARPING_DATA warpingData;
    NvAPI_ShortString estring;
    int maxNumVertices = 0;

    // for loading blending images
    std::vector<unsigned char> image;
    unsigned width, height;

    printf("App Version: 1.2\n");

    // Initialize NVAPI, get GPU handles, etc.
    error = NvAPI_Initialize();
    ZeroMemory(&nvGPUHandles, sizeof(nvGPUHandles));
    error = NvAPI_EnumPhysicalGPUs(nvGPUHandles, &gpuCount);

    //std::vector<std::vector<std::size_t>> display_ids{ {0x80061086, 0x80061087}, {0x82061086, 0x82061087} };
    std::vector < std::size_t > known_ids { 0x80061086, 0x80061087, 0x82061086, 0x82061087 };

    // a mapping from displayId to (row,column) location of projector
    std::map<int, pair<int, int>> locationsOfDisplay;
    locationsOfDisplay[known_ids[0]] = make_pair(0, 0);
    locationsOfDisplay[known_ids[1]] = make_pair(0, 1);

    locationsOfDisplay[known_ids[2]] = make_pair(1, 0);
    locationsOfDisplay[known_ids[3]] = make_pair(1, 1);

    for (auto id : known_ids)
        printf("known id: 0x%08x \n", id);

    // At this point we have a list of accessible physical nvidia gpus in the system.
    // Loop over all gpus
    for (gpu = 0; gpu < gpuCount; gpu++) {
        NvU32 dispIdCount = 0;

        // Query the active physical display connected to each gpu.
        error = NvAPI_GPU_GetConnectedDisplayIds(nvGPUHandles[gpu], NULL, &dispIdCount, 0);
        printf("Display count %d\n", dispIdCount);
        if ((error != NVAPI_OK) || (dispIdCount == 0)) {
            NvAPI_GetErrorMessage(error, estring);
            printf("NvAPI_GPU_GetConnectedDisplayIds: %s\n", estring);
            return error;
        }

        NV_GPU_DISPLAYIDS *dispIds = NULL;
        dispIds = new NV_GPU_DISPLAYIDS[dispIdCount];
        dispIds->version = NV_GPU_DISPLAYIDS_VER;
        error = NvAPI_GPU_GetConnectedDisplayIds(nvGPUHandles[gpu], dispIds, &dispIdCount, 0);
        if (error != NVAPI_OK) {
            delete[] dispIds;
            NvAPI_GetErrorMessage(error, estring);
            printf("NvAPI_GPU_GetConnectedDisplayIds: %s\n", estring);
            return error;
        }

        //for (NvU32 dispIndex = 0; (dispIndex < dispIdCount) && dispIds[dispIndex].isActive; dispIndex++)
        for (NvU32 dispIndex = 0; dispIndex < dispIdCount; dispIndex++) {
            NV_SCANOUT_INFORMATION scanInfo;

            ZeroMemory(&scanInfo, sizeof(NV_SCANOUT_INFORMATION));
            scanInfo.version = NV_SCANOUT_INFORMATION_VER;

            printf("GPU %d, displayId 0x%08x\n", gpu, dispIds[dispIndex].displayId);
            std::cout << "active: " << dispIds[dispIndex].isActive << std::endl;
        }
        printf("next gpu\n");
    }

    printf("scanning completed\n\n");

    for (gpu = 0; gpu < gpuCount; gpu++) {
        NvU32 dispIdCount = 0;

        // Query the active physical display connected to each gpu.
        error = NvAPI_GPU_GetConnectedDisplayIds(nvGPUHandles[gpu], NULL, &dispIdCount, 0);
        if ((error != NVAPI_OK) || (dispIdCount == 0)) {
            NvAPI_GetErrorMessage(error, estring);
            printf("NvAPI_GPU_GetConnectedDisplayIds: %s\n", estring);
            printf("Display count %d\n", dispIdCount);
            return error;
        }

        NV_GPU_DISPLAYIDS *dispIds = NULL;
        dispIds = new NV_GPU_DISPLAYIDS[dispIdCount];
        dispIds->version = NV_GPU_DISPLAYIDS_VER;

        error = NvAPI_GPU_GetConnectedDisplayIds(nvGPUHandles[gpu], dispIds, &dispIdCount, 0);
        if (error != NVAPI_OK) {
            delete[] dispIds;
            NvAPI_GetErrorMessage(error, estring);
            printf("NvAPI_GPU_GetConnectedDisplayIds: %s\n", estring);
            return error;
        }

        // Loop through all the displays
        for (NvU32 dispIndex = 0; (dispIndex < dispIdCount) && dispIds[dispIndex].isActive; dispIndex++) {
            printf("trying id: 0x%08x \n", dispIds[dispIndex].displayId);
            if (std::count(known_ids.begin(), known_ids.end(), static_cast<std::size_t>(dispIds[dispIndex].displayId))
                    == 0) {
                printf("not in the known list\n");
                continue;
            }

            NV_SCANOUT_INFORMATION scanInfo;

            ZeroMemory(&scanInfo, sizeof(NV_SCANOUT_INFORMATION));
            scanInfo.version = NV_SCANOUT_INFORMATION_VER;

            printf("GPU %d, displayId 0x%08x\n", gpu, dispIds[dispIndex].displayId);

            // Query the desktop size and display location in it
            error = NvAPI_GPU_GetScanoutConfigurationEx(dispIds[dispIndex].displayId, &scanInfo);
            if (error != NVAPI_OK) {
                NvAPI_GetErrorMessage(error, estring);
                printf("NvAPI_GPU_GetScanoutConfiguration: %s\n", estring);
                return error;
            }

            // Desktop -- the size & location of the virtual desktop in Windows, in a Mosaic this will include all displays and overalp
            printf("DesktopRect: sX = %6d, sY = %6d, sWidth = %6d sHeight = %6d\n", scanInfo.sourceDesktopRect.sX,
                    scanInfo.sourceDesktopRect.sY, scanInfo.sourceDesktopRect.sWidth,
                    scanInfo.sourceDesktopRect.sHeight);

            // source Viewport -- where in the desktop this selected display is
            printf("ViewportRect: sX = %6d, sY = %6d, sWidth = %6d sHeight = %6d\n", scanInfo.sourceViewportRect.sX,
                    scanInfo.sourceViewportRect.sY, scanInfo.sourceViewportRect.sWidth,
                    scanInfo.sourceViewportRect.sHeight);

            // What resolution is the display
            printf("Display Scanout Rect: sX = %6d, sY = %6d, sWidth = %6d sHeight = %6d\n",
                    scanInfo.targetViewportRect.sX, scanInfo.targetViewportRect.sY, scanInfo.targetViewportRect.sWidth,
                    scanInfo.targetViewportRect.sHeight);

            // texture coordinates
            // Computing the texture coordinates is different if we are in Mosaic vs extended displays, so check to see if Mosaic is running
            NV_MOSAIC_TOPO_BRIEF topo;
            topo.version = NVAPI_MOSAIC_TOPO_BRIEF_VER;

            NV_MOSAIC_DISPLAY_SETTING dispSetting;
            dispSetting.version = NVAPI_MOSAIC_DISPLAY_SETTING_VER;

            NvS32 overlapX, overlapY;
            NvU32 display_id = dispIds[dispIndex].displayId;

            // Query the current Mosaic topology
            error = NvAPI_Mosaic_GetCurrentTopo(&topo, &dispSetting, &overlapX, &overlapY);
            if (error != NVAPI_OK) {
                NvAPI_GetErrorMessage(error, estring);
                printf("NvAPI_GPU_GetCurrentTopo: %s\n", estring);
                return error;
            }

            int desktop = gpu;  // TEMPORARY for TRIP4 installation only - each GPU has one separate mosaic desktop
            int row = locationsOfDisplay[dispIds[dispIndex].displayId].first;
            int col = locationsOfDisplay[dispIds[dispIndex].displayId].second;

            printf("Warping projector at ROW: %d, COL: %d\n", row, col);

            // warp vertices are defined in scanoutRect coordinates
            auto scanoutRect = scanInfo.targetViewportRect;

            float dstWidth = scanoutRect.sWidth / 2.0f;
            float dstHeight = scanoutRect.sHeight;
            float dstXShift = dstWidth / 2.0f;
            float dstYShift = dstHeight / 2.0f;
            float dstLeft = (float) scanoutRect.sX + dstXShift;
            float dstTop = (float) scanoutRect.sY;

            // Triangle strip with 4 vertices
            // The vertices are given as a 2D vertex strip because the warp
            // is a 2d operation. To be able to emulate 3d perspective correction,
            // the texture coordinate contains 4 components, which needs to be
            // adjusted to get this correction.
            //
            // A trapezoid needs 4 vertices.
            // Format is xy for the vertex + yurq for the texture coordinate.
            // So we need 24 floats for that.
            //float vertices [4*6];
            //
            //  (0)  ----------------  (2)
            //       |             / |
            //       |            /  |
            //       |           /   |
            //       |          /    |
            //       |         /     |
            //       |        /      |
            //       |       /       |
            //       |      /        |
            //       |     /         |
            //       |    /          |
            //       |   /           |
            //       |  /            |
            //       | /             |
            //   (1) |---------------- (3)
            //

            // texture coordinates
            // warp texture coordinates are defined in desktopRect coordinates

            auto desktopRect = scanInfo.sourceDesktopRect;
            float srcLeft = (float) desktopRect.sX;
            float srcTop = (float) desktopRect.sY;
            float srcWidth = desktopRect.sWidth;
            float srcHeight = desktopRect.sHeight;

            RectCoords target_coords = read_warping_vertices(gpu, row, col);

            printf("Warping projector at ROW: %d, COL: %d\n", row, col);
            std::vector<float> vertices = get_warping_vertices(srcLeft, srcTop, srcWidth, srcHeight, target_coords);
            warp_display(scanInfo, display_id, vertices, warpingData);
            cout << "Initial warp finished." << endl;

            if (interactive) {
                char c = ' ';
                while (c != 'q') {
                    cout << "Select vertex (0 = TL, 1 = BL, 2 = TR, 3 = BR, q = EXIT): ";
                    cin >> c;
                    Vector2f* vertex = nullptr;
                    switch (c) {
                    case '0':
                        vertex = &target_coords.tl;
                        break;
                    case '1':
                        vertex = &target_coords.tr;
                        break;
                    case '2':
                        vertex = &target_coords.bl;
                        break;
                    case '3':
                        vertex = &target_coords.br;
                        break;
                    default:
                        continue;
                    }
                    move_vertex(vertex);
                    printf("Warping projector at ROW: %d, COL: %d\n", row, col);
                    cout << "Warping vertex " << c << " to " << vertex->transpose() << endl;
                    std::vector<float> vertices = get_warping_vertices(srcLeft, srcTop, srcWidth, srcHeight, target_coords);
                    warp_display(scanInfo, display_id, vertices, warpingData);
                }
            }
            else {
                cout << "non-interactive mode (add any command line parameter to run interactively)" << endl;
            }

            /*
             // -----------------------------------------------------------------------------
             // BLENDING
             // -----------------------------------------------------------------------------

             NV_SCANOUT_INTENSITY_DATA intensityData;

             image.clear();
             string blend_filename = "blend_(" + to_string(row) + ", " + to_string(col) + ").png";
             unsigned lodePng_error = lodepng::decode(image, width, height, blend_filename);
             if (lodePng_error) printf("LODEPNG ERROR %s\n ", lodepng_error_text(lodePng_error));

             // copy values from png, skip every 4th value - we don't want the alpha channel
             std::vector<float> intensityTexture_vec;
             for (int i = 0; i < image.size(); i++) {
             if ((i + 1) % 4 != 0) {
             intensityTexture_vec.push_back(image[i] / 255.0f);
             }
             }

             float* intensityTexture = &intensityTexture_vec[0];

             intensityData.version = NV_SCANOUT_INTENSITY_DATA_VER;
             intensityData.width = 1920;
             intensityData.height = 1080;
             intensityData.blendingTexture = intensityTexture;

             // do not want to use an offset texture
             intensityData.offsetTexture = NULL;
             intensityData.offsetTexChannels = 1;

             // this call does the intensity map
             error = NvAPI_GPU_SetScanoutIntensity(dispIds[dispIndex].displayId, &intensityData, &sticky);

             if (error != NVAPI_OK)
             {
             NvAPI_GetErrorMessage(error, estring);
             printf("NvAPI_GPU_SetScanoutIntensity: %s\n", estring);
             return error;
             }
             */

        } //end of for displays

        delete[] dispIds;

    } //end of loop gpus

}


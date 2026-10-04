#include "gui.h"

#include <GLFW/glfw3.h>
#include <map>
#include <sstream>
#include <fstream>
#include <iostream>

#include <random>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <nfd.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "tabs/tab_list.h"
#include "tab_registry.h"

namespace fs = std::filesystem;

namespace {
constexpr const char* FONT_PATH   = "assets/fonts/Roboto.ttf";
constexpr const char* LAYOUTS_DIR = "layouts";

// Ensure the layout directory exists
void EnsureLayoutsDirectoryExists() {
    if (!fs::exists(LAYOUTS_DIR)) {
        fs::create_directories(LAYOUTS_DIR);
    }
}
} // namespace

void GUI::SaveLayoutDialog() {
    EnsureLayoutsDirectoryExists();

    nfdchar_t* savePath           = nullptr;
    nfdfilteritem_t filterItem[1] = {{"Layout Files", "json"}};

    // Set default directory to the layouts folder
    std::string defaultPath = fs::absolute(LAYOUTS_DIR).string();

    nfdresult_t result =
        NFD_SaveDialog(&savePath, filterItem, 1, defaultPath.c_str(), "my_layout.json");
    if (result == NFD_OKAY) {
        SaveStateToFile(savePath);
        NFD_FreePath(savePath);
    }
}

void GUI::LoadLayoutDialog() {
    EnsureLayoutsDirectoryExists();

    nfdchar_t* openPath           = nullptr;
    nfdfilteritem_t filterItem[1] = {{"Layout Files", "json"}};

    std::string defaultPath = fs::absolute(LAYOUTS_DIR).string();

    nfdresult_t result = NFD_OpenDialog(&openPath, filterItem, 1, defaultPath.c_str());
    if (result == NFD_OKAY) {
        // Clear existing tabs before loading a custom layout
        m_tabs.clear();
        LoadStateFromFile(openPath);
        NFD_FreePath(openPath);
    }
}

struct MenuNode {
    std::map<std::string, MenuNode> submenus;
    std::vector<const TabInfo*> items;
};

void GUI::Render() {
    // Start a new ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Enable the dockspace
    ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());

    // Render top menu bar
    RenderMainMenuBar();

    // Render registered tab objects and remove closed ones
    for (auto it = m_tabs.begin(); it != m_tabs.end();) {
        (*it)->Render();

        if (!(*it)->IsOpen()) {
            it = m_tabs.erase(it);
        } else {
            ++it;
        }
    }

    // Finalize ImGui draw data
    ImGui::Render();

    int width  = 0;
    int height = 0;
    glfwGetFramebufferSize(m_window, &width, &height);

    // Skip rendering if there is no framebuffer (ex: minimized)
    if (width <= 0 || height <= 0)
        return;

    glViewport(0, 0, width, height);

    // Clear the framebuffer before drawing the UI
    glClearColor(0.025f, 0.032f, 0.042f, 1.0f);

    glClear(GL_COLOR_BUFFER_BIT);

    // Submit the ImGui draw data to OpenGL
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

GUI::~GUI() {
    Shutdown();
}

bool GUI::Initialize(GLFWwindow* window) {
    if (!window)
        return false;

    m_window = window;

    // Dear ImGui
    IMGUI_CHECKVERSION();

    ImGui::CreateContext();

    ImGuiIO& io    = ImGui::GetIO();
    io.IniFilename = "imgui.ini";
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui::StyleColorsDark();

    // DPI Scaling
    float xscale = 1.0f;
    float yscale = 1.0f;

    glfwGetWindowContentScale(window, &xscale, &yscale);

    ApplyStyle(xscale);

    // Font Loading
    if (!io.Fonts->AddFontFromFileTTF(FONT_PATH, 16.0f * xscale)) {
        io.Fonts->AddFontDefault();
    }

    // ImGui Backends
    if (!ImGui_ImplGlfw_InitForOpenGL(window, true)) {
        Shutdown();
        return false;
    }

    if (!ImGui_ImplOpenGL3_Init("#version 330")) {
        Shutdown();
        return false;
    }

    LoadStateFromFile("layout.json");

    return true;
}

void GUI::Shutdown() {
    SaveStateToFile("layout.json");

    // Shut down ImGui backends before destroying the context
    if (ImGui::GetCurrentContext()) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();

        ImGui::DestroyContext();
    }

    m_window = nullptr;
}

// Helper function to split category paths like "Tools/Debug/Console" by '/'
std::vector<std::string> SplitPath(const std::string& path, char delimiter = '/') {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(path);
    while (std::getline(tokenStream, token, delimiter)) {
        if (!token.empty()) {
            tokens.push_back(token);
        }
    }
    return tokens;
}

// ID Generator
static std::string GetRandomId() {
    static std::random_device rd;
    static std::mt19937_64 gen(rd());
    static std::uniform_int_distribution<uint64_t> dis;

    std::stringstream ss;
    ss << std::hex << std::setfill('0') << std::setw(16) << dis(gen);
    return ss.str();
}

// Recursive function to draw ImGui menus
void RenderSubmenuTree(const MenuNode& node, std::vector<std::unique_ptr<ITab>>& activeTabs) {
    for (const auto* info : node.items) {
        std::string label = "Open " + info->displayName;
        if (ImGui::MenuItem(label.c_str())) {
            activeTabs.push_back(info->factory(GetRandomId()));
        }
    }

    for (const auto& [categoryName, childNode] : node.submenus) {
        if (ImGui::BeginMenu(categoryName.c_str())) {
            RenderSubmenuTree(childNode, activeTabs);
            ImGui::EndMenu();
        }
    }
}

void GUI::RenderMainMenuBar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Load Layout...")) {
                LoadLayoutDialog();
            }
            if (ImGui::MenuItem("Save Layout As...")) {
                SaveLayoutDialog();
            }

            ImGui::Separator();

            if (ImGui::MenuItem("Exit")) {
                glfwSetWindowShouldClose(m_window, true);
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Windows")) {
            // Build hierarchy from registered tabs
            MenuNode root;
            const auto& registry = TabRegistry::Instance().GetRegisteredTabs();

            for (const auto& info : registry) {
                std::vector<std::string> path = SplitPath(info.category);
                MenuNode* current             = &root;

                for (const auto& folder : path) {
                    current = &current->submenus[folder];
                }
                current->items.push_back(&info);
            }

            // Render the menu hierarchy
            RenderSubmenuTree(root, m_tabs);

            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }
}

void GUI::SaveStateToFile(const std::string& filepath) const {
    std::ofstream file(filepath);
    if (!file.is_open())
        return;

    file << "{\n  \"open_tabs\": [\n";
    for (size_t i = 0; i < m_tabs.size(); ++i) {
        file << "    { \"title\": \"" << m_tabs[i]->GetTitle() << "\", \"instance_id\": \""
             << m_tabs[i]->GetInstanceId() << "\" }";
        if (i + 1 < m_tabs.size())
            file << ",";
        file << "\n";
    }
    file << "  ]\n}\n";
}

void GUI::LoadStateFromFile(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open())
        return;

    std::string line;
    const auto& registry = TabRegistry::Instance();

    while (std::getline(file, line)) {
        size_t titlePos = line.find("\"title\": \"");
        size_t idPos    = line.find("\"instance_id\": \"");

        if (titlePos != std::string::npos) {
            size_t titleStart = titlePos + 10;
            size_t titleEnd   = line.find("\"", titleStart);
            std::string title = line.substr(titleStart, titleEnd - titleStart);

            std::string instanceId = "0";
            if (idPos != std::string::npos) {
                size_t idStart = idPos + 16;
                size_t idEnd   = line.find("\"", idStart);
                instanceId     = line.substr(idStart, idEnd - idStart);
            }

            for (const auto& info : registry.GetRegisteredTabs()) {
                if (info.displayName == title || info.id == title) {
                    m_tabs.push_back(info.factory(instanceId));
                    break;
                }
            }
        }
    }
}

void GUI::ApplyStyle(float scale) {
    ImGuiStyle& style = ImGui::GetStyle();

    // Geometry
    style.WindowPadding    = ImVec2(18.0f, 16.0f);
    style.FramePadding     = ImVec2(11.0f, 8.0f);
    style.ItemSpacing      = ImVec2(10.0f, 10.0f);
    style.ItemInnerSpacing = ImVec2(8.0f, 6.0f);

    style.IndentSpacing = 22.0f;

    style.ScrollbarSize = 11.0f;
    style.GrabMinSize   = 10.0f;

    // Rounding
    style.WindowRounding    = 6.0f;
    style.ChildRounding     = 5.0f;
    style.FrameRounding     = 4.0f;
    style.PopupRounding     = 5.0f;
    style.ScrollbarRounding = 5.0f;
    style.GrabRounding      = 4.0f;
    style.TabRounding       = 4.0f;

    // Borders
    style.WindowBorderSize = 1.0f;
    style.ChildBorderSize  = 1.0f;
    style.PopupBorderSize  = 1.0f;

    style.FrameBorderSize = 1.0f;
    style.TabBorderSize   = 0.0f;

    // Text Alignment
    style.WindowTitleAlign = ImVec2(0.0f, 0.5f);
    style.ButtonTextAlign  = ImVec2(0.5f, 0.5f);

    // Palette
    ImVec4* c = style.Colors;

    // Text
    c[ImGuiCol_Text] = ImVec4(0.91f, 0.93f, 0.96f, 1.00f);

    c[ImGuiCol_TextDisabled] = ImVec4(0.45f, 0.50f, 0.56f, 1.00f);

    c[ImGuiCol_TextSelectedBg] = ImVec4(0.10f, 0.43f, 0.58f, 0.45f);

    // Window Background
    c[ImGuiCol_WindowBg] = ImVec4(0.075f, 0.085f, 0.100f, 1.00f);

    c[ImGuiCol_ChildBg] = ImVec4(0.095f, 0.108f, 0.125f, 1.00f);

    c[ImGuiCol_PopupBg] = ImVec4(0.085f, 0.098f, 0.115f, 0.99f);

    // Borders
    c[ImGuiCol_Border] = ImVec4(0.27f, 0.31f, 0.36f, 1.00f);

    c[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);

    // Input Fields
    c[ImGuiCol_FrameBg] = ImVec4(0.055f, 0.068f, 0.082f, 1.00f);

    c[ImGuiCol_FrameBgHovered] = ImVec4(0.075f, 0.095f, 0.115f, 1.00f);

    c[ImGuiCol_FrameBgActive] = ImVec4(0.085f, 0.115f, 0.140f, 1.00f);

    // Buttons
    c[ImGuiCol_Button] = ImVec4(0.105f, 0.125f, 0.150f, 1.00f);

    c[ImGuiCol_ButtonHovered] = ImVec4(0.125f, 0.205f, 0.250f, 1.00f);

    c[ImGuiCol_ButtonActive] = ImVec4(0.145f, 0.285f, 0.350f, 1.00f);

    // Separators
    c[ImGuiCol_Separator] = ImVec4(0.27f, 0.31f, 0.36f, 1.00f);

    c[ImGuiCol_SeparatorHovered] = ImVec4(0.20f, 0.55f, 0.72f, 1.00f);

    c[ImGuiCol_SeparatorActive] = ImVec4(0.25f, 0.68f, 0.85f, 1.00f);

    // Accent / Controls
    c[ImGuiCol_CheckMark] = ImVec4(0.20f, 0.72f, 0.92f, 1.00f);

    c[ImGuiCol_SliderGrab] = ImVec4(0.12f, 0.55f, 0.73f, 1.00f);

    c[ImGuiCol_SliderGrabActive] = ImVec4(0.20f, 0.72f, 0.92f, 1.00f);

    // Tabs
    c[ImGuiCol_Tab] = ImVec4(0.065f, 0.080f, 0.095f, 1.00f);

    c[ImGuiCol_TabHovered] = ImVec4(0.10f, 0.25f, 0.32f, 1.00f);

    c[ImGuiCol_TabActive] = ImVec4(0.09f, 0.19f, 0.24f, 1.00f);

    c[ImGuiCol_TabUnfocused] = ImVec4(0.055f, 0.068f, 0.080f, 1.00f);

    c[ImGuiCol_TabUnfocusedActive] = ImVec4(0.075f, 0.125f, 0.155f, 1.00f);

    // Title Bars
    c[ImGuiCol_TitleBg] = ImVec4(0.045f, 0.055f, 0.068f, 1.00f);

    c[ImGuiCol_TitleBgActive] = ImVec4(0.070f, 0.095f, 0.120f, 1.00f);

    c[ImGuiCol_TitleBgCollapsed] = ImVec4(0.040f, 0.048f, 0.058f, 1.00f);

    // Scrollbar
    c[ImGuiCol_ScrollbarBg] = ImVec4(0.045f, 0.052f, 0.062f, 1.00f);

    c[ImGuiCol_ScrollbarGrab] = ImVec4(0.16f, 0.19f, 0.225f, 1.00f);

    c[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.21f, 0.26f, 0.30f, 1.00f);

    c[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.25f, 0.32f, 0.37f, 1.00f);

    // Resize Grips
    c[ImGuiCol_ResizeGrip] = ImVec4(0.12f, 0.42f, 0.55f, 0.25f);

    c[ImGuiCol_ResizeGripHovered] = ImVec4(0.16f, 0.58f, 0.73f, 0.65f);

    c[ImGuiCol_ResizeGripActive] = ImVec4(0.20f, 0.70f, 0.86f, 0.90f);

    // Menu Bar
    c[ImGuiCol_MenuBarBg] = ImVec4(0.055f, 0.065f, 0.078f, 1.00f);

    // Empty DockSpace Background
    c[ImGuiCol_DockingEmptyBg] = ImVec4(0.040f, 0.048f, 0.058f, 1.00f);

    c[ImGuiCol_DockingPreview] = ImVec4(0.200f, 0.720f, 0.920f, 0.35f);

    // Tabs & Tab Headers
    c[ImGuiCol_Tab] = ImVec4(0.065f, 0.080f, 0.095f, 1.00f);

    c[ImGuiCol_TabHovered] = ImVec4(0.120f, 0.220f, 0.280f, 1.00f);

    c[ImGuiCol_TabActive] = ImVec4(0.095f, 0.125f, 0.150f, 1.00f);

    c[ImGuiCol_TabUnfocused] = ImVec4(0.050f, 0.060f, 0.072f, 1.00f);

    c[ImGuiCol_TabUnfocusedActive] = ImVec4(0.075f, 0.095f, 0.115f, 1.00f);

    c[ImGuiCol_TabSelectedOverline] = ImVec4(0.200f, 0.720f, 0.920f, 1.00f);

    style.ScaleAllSizes(scale);
}
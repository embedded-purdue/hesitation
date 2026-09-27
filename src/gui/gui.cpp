#include "gui.h"

#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

namespace {
constexpr const char* FONT_PATH = "assets/fonts/Roboto.ttf";
}

void GUI::Render() {
    // Start a new ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();

    ImGui::NewFrame();

    // Application UI
    ImGui::Begin("Main Menu");

    ImGui::Text("Hello from the GUI!");

    static float slider_val = 0.5f;

    ImGui::SliderFloat("Value", &slider_val, 0.0f, 1.0f);

    if (ImGui::Button("Click Me")) {
        // Handle button action
    }

    ImGui::End();

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

    ImGuiIO& io = ImGui::GetIO();

    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

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

    return true;
}

void GUI::Shutdown() {
    // Shut down ImGui backends before destroying the context
    if (ImGui::GetCurrentContext()) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();

        ImGui::DestroyContext();
    }

    m_window = nullptr;
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

    style.ScaleAllSizes(scale);
}
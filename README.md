# Hesitation
NEP Testbed

![Cover](docs/main_pic.png)

---

## 🚀 Getting Started

### Prerequisites

* **CMake** (3.20 or higher)
* **Compiler**: C++17 compatible (`clang++`, `g++`, or MSVC)
* **OpenGL Development Libraries** *(likely already installed)*
  * **Ubuntu/Debian:** `sudo apt install build-essential cmake libgl1-mesa-dev xorg-dev`
  * **macOS:** `xcode-select --install`
  * **Windows:** Visual Studio (Desktop C++ workload) or MSYS2

### Required VS Code Extensions

Open VS Code and install:

1. **C/C++** (`ms-vscode.cpptools`) — Intellisense, syntax highlighting, and debugging
2. **CMake Tools** (`ms-vscode.cmake-tools`) — Build automation and configuration
3. **Clang-Format** (`xaver.clang-format`) — Automatic code formatting

---

## 🛠️ Building & Running

1. **Open Project:** Launch VS Code and open the repository root (`File` → `Open Folder...`).
2. **Select Kit:** Choose your C++ compiler when prompted by CMake Tools in the bottom status bar.
   > *If not prompted:* Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on macOS), type `CMake: Select a Kit`, and pick your compiler.
3. **Build:** Click **Build** on the status bar (or press `F7`).
4. **Run:** Click the **Play** button on the status bar (or press `Shift+F5`).

---

## 🎨 Code Formatting

This repository provides pre-configured `.clang-format` and `.vscode/settings.json` files. 

Once the **Clang-Format** extension is installed, formatting automatically applies on save—no extra setup required.

---

## 🏗️ Application Architecture

The project follows a modular structure under the `src/` directory with strict separation of concerns:

| Subsystem / File | Responsibilities |
| :--- | :--- |
| **`src/gui/`** | UI components, Dear ImGui rendering, and user input handling. |
| **`src/physics/`** | Orbital simulation engine, timing/scheduling, and reactor/engine/thermo simulations. |
| **`src/hardware/`** | Hardware abstraction, serial protocols, and physical device interfaces. |
| **`src/main.h`** | Central process manager strictly responsible for owning and managing subsystem lifecycles. |

### Subsystem Communication & Boundaries
* **Strict Encapsulation:** Regular classes within `gui/`, `physics/`, or `hardware/` are strictly isolated and must **never** communicate directly with other modules.
* **Message-Passing Architecture:** All inter-module communication is explicitly planned, documented, and handled exclusively by each module's main entry class using **Protocol Buffers (Protobuf)** messages.
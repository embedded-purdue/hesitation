import os
import platform
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PYTHON_DIR = ROOT / "app" / "src" / "python"
VENV_DIR = PYTHON_DIR / ".venv"

if platform.system() == "Windows":
    PYTHON = VENV_DIR / "Scripts" / "python.exe"
else:
    PYTHON = VENV_DIR / "bin" / "python"

def run(command):
    subprocess.run(command, check=True)

def main():
    print("=== Hesitation Developer Setup ===")

    if not PYTHON.exists():
        print("Creating Python virtual environment...")
        run([sys.executable, "-m", "venv", str(VENV_DIR)])

    print("Upgrading pip...")
    run([str(PYTHON), "-m", "pip", "install", "--upgrade", "pip"])

    requirements = PYTHON_DIR / "requirements.txt"

    if requirements.exists():
        print("Installing Python dependencies...")
        run([
            str(PYTHON),
            "-m",
            "pip",
            "install",
            "-r",
            str(requirements)
        ])

    print("\nPython environment initialized successfully!")
    print(f"Interpreter: {PYTHON}")

if __name__ == "__main__":
    main()

# SSH Dashboard V3 - Enhanced File Management Edition

🚀 **Advanced SSH dashboard with comprehensive file management capabilities for your Mini Rover project!**

## ✨ New Features in V3

### 🌳 Dual-Pane File Browser
- **Local file browser** - Navigate your local files easily
- **Remote file browser** - Browse rover files in a tree structure
- **Side-by-side layout** - See both local and remote files simultaneously

### 📁 Enhanced File Operations
- **Smart upload buttons** - One-click upload to Sensors/, Navigation/, Main/, Behaviors/
- **Context menus** - Right-click for upload, download, delete, copy path
- **Progress tracking** - Visual progress bars for file operations
- **Backup creation** - Automatic backups before overwriting files
- **Batch operations** - Select and operate on multiple files

### 🎯 Smart Deployment Features
- **Module-specific uploads** - Direct upload to rover's modular directories
- **Overwrite protection** - Confirm before replacing important files
- **Quick navigation** - Breadcrumb paths and easy directory switching

### 💻 Enhanced Terminal
- **Copy/paste support** - Full clipboard integration (Ctrl+C, Ctrl+V, Ctrl+A)
- **Quick commands** - One-click buttons for common operations
- **Click-to-navigate** - Right-click files → "Open in Terminal"
- **Command history** - Better command management

### 🚀 Program Launcher
- **Program discovery** - Automatically finds Python programs on the rover
- **One-click execution** - Select and run programs with virtual environment support
- **Program management** - Start, stop, and monitor running programs
- **Program info** - View descriptions and details for each program

### 🎨 Improved UI/UX
- **Tabbed interface** - Organized into Connection & Terminal, File Management, Settings
- **Dark/Light themes** - Toggle between themes
- **Better layouts** - More intuitive and spacious design
- **Status indicators** - Clear connection and operation status

## 🚀 Quick Start

1. **Launch the dashboard:**
   ```bash
   cd "Dashboard/Dashboard V3 - Enhanced File Management"
   ./run_v3.sh
   ```

2. **Connect to your rover:**
   - Enter rover IP (default: 192.168.254.65)
   - Username: jay
   - Password: [your password]
   - Click "Connect"

3. **Use the enhanced features:**
   - Browse local files on the left pane
   - Browse rover files on the right pane
   - Use smart upload buttons for quick deployment
   - Right-click for context menus
   - Select and run programs from the launcher panel
   - Use program management tools to delete programs

## 📋 Key Improvements Over V2

| Feature | V2 | V3 |
|---------|----|----|
| File Browser | Basic upload dialog | Dual-pane tree browser |
| Upload | Single file/folder | Smart deployment + batch |
| Navigation | Command line only | Visual tree + click navigation |
| Progress | None | Visual progress bars |
| Backup | None | Automatic backup creation |
| UI Layout | Single window | Tabbed interface |
| Context Menus | None | Right-click operations |
| File Operations | Limited | Upload, download, delete, compare |
| Program Management | None | Launcher panel + delete tools |

## 🛠️ Smart Upload Buttons

- **📡 Upload to Sensors/** - Direct upload to rover's Sensors directory
- **🧭 Upload to Navigation/** - Upload navigation modules
- **🤖 Upload to Main/** - Upload core system files  
- **🎯 Upload to Behaviors/** - Upload autonomous behavior modules

## 🎨 Themes

Toggle between light and dark themes in the Settings tab for comfortable usage in any environment.

## 📝 Coming Soon

- **Drag & drop uploads** - Drag files from local to remote pane
- **File comparison** - Visual diff between local and remote files
- **Download functionality** - Download files from rover to local
- **Synchronized scrolling** - Keep panes aligned
- **File search** - Search for files across directories

## 🐛 Feedback

Test the new features and provide feedback! This is a complete rewrite with much better file management capabilities.

---
**Dashboard V3 - Enhanced File Management Edition**  
*Making rover file management as easy as drag and drop!* 🤖✨
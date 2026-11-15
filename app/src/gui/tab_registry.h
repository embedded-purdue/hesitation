#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <type_traits>
#include "tab.h"

struct TabInfo {
    std::string id{};
    std::string category{};
    std::string displayName{};
    std::function<std::unique_ptr<ITab>(const std::string& instanceId)> factory{};

    // Helper method: allows calling factory() without passing an instance ID explicitly
    std::unique_ptr<ITab> Create(const std::string& instanceId = "0") const {
        return factory ? factory(instanceId) : nullptr;
    }
};

class TabRegistry {
  public:
    static TabRegistry& Instance() {
        static TabRegistry instance;
        return instance;
    }

    template <typename T>
    void RegisterTab(const std::string& category, const std::string& displayName,
                     const std::string& id) {
        TabInfo info{};
        info.id          = id;
        info.category    = category;
        info.displayName = displayName;

        // Automatically supports constructors taking std::string or default constructors
        info.factory = [](const std::string& instanceId) -> std::unique_ptr<ITab> {
            if constexpr (std::is_constructible_v<T, std::string>) {
                return std::make_unique<T>(instanceId);
            } else {
                return std::make_unique<T>();
            }
        };

        m_tabs.push_back(std::move(info));
    }

    const std::vector<TabInfo>& GetRegisteredTabs() const {
        return m_tabs;
    }

  private:
    std::vector<TabInfo> m_tabs;
};

#define REGISTER_TAB(TabClass, Category, DisplayName)                                              \
    static struct TabClass##Register {                                                             \
        TabClass##Register() {                                                                     \
            TabRegistry::Instance().RegisterTab<TabClass>(Category, DisplayName, #TabClass);       \
        }                                                                                          \
    } global_##TabClass##Register;
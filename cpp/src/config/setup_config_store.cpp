#include "config/setup_config_store.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <stdexcept>

namespace fitra::config {

namespace {
bool ends_with(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() &&
           s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}
}  // namespace

SetupConfigStore::SetupConfigStore(MainOptions seed, std::string union_path,
                                   std::string named_dir)
    : draft_(std::move(seed)),
      union_path_(std::move(union_path)),
      named_dir_(std::move(named_dir)) {}

MainOptions SetupConfigStore::draft() const {
    std::lock_guard<std::mutex> lk{mu_};
    return draft_;
}

void SetupConfigStore::set_draft(const MainOptions& opts) {
    std::lock_guard<std::mutex> lk{mu_};
    draft_ = opts;
}

bool SetupConfigStore::validate_draft(std::string& err) const {
    MainOptions copy;
    {
        std::lock_guard<std::mutex> lk{mu_};
        copy = draft_;
    }
    // Validate in daemon form: the union config need not be runnable yet (no
    // cameras/engines required mid-setup), but ranges/enums must be sane. The
    // daemon flags are CLI-only, never written, so setting daemon here only
    // selects the relaxed run-form path in validate_options.
    copy.daemon = true;
    copy.flow_managed = false;
    copy.setup_mode = false;
    try {
        validate_options(copy);
        return true;
    } catch (const std::exception& e) {
        err = e.what();
        return false;
    }
}

bool SetupConfigStore::write_union(std::string& err) {
    // Never clobber a tracked template: --config must point at a writable
    // runtime config (the daemon bootstraps a missing one from the .example).
    if (ends_with(union_path_, ".example")) {
        err = "refusing to overwrite the tracked template '" + union_path_ +
              "'; point --config at a writable runtime config "
              "(e.g. configs/session.yaml) instead";
        return false;
    }
    MainOptions copy;
    {
        std::lock_guard<std::mutex> lk{mu_};
        copy = draft_;
    }
    // Persist absolute paths so the daemon's mode children resolve engines /
    // calib artifacts unambiguously (they open CWD-relative; the setup module
    // shares the daemon CWD, so absolutize here against that same base).
    absolutize_config_paths(copy);
    try {
        save_main_config(union_path_, copy);
        return true;
    } catch (const std::exception& e) {
        err = e.what();
        return false;
    }
}

bool SetupConfigStore::valid_name(const std::string& name) {
    if (name.empty() || name.size() > 64) return false;
    return std::all_of(name.begin(), name.end(), [](unsigned char c) {
        return std::isalnum(c) || c == '_' || c == '-';
    });
}

std::string SetupConfigStore::named_path(const std::string& name) const {
    return (std::filesystem::path{named_dir_} / (name + ".yaml")).string();
}

bool SetupConfigStore::save_named(const std::string& name, std::string& err) {
    if (!valid_name(name)) {
        err = "invalid config name (use letters, digits, _ or -, max 64 chars)";
        return false;
    }
    MainOptions copy;
    {
        std::lock_guard<std::mutex> lk{mu_};
        copy = draft_;
    }
    absolutize_config_paths(copy);  // same as write_union: persist absolute paths
    try {
        std::error_code ec;
        std::filesystem::create_directories(named_dir_, ec);  // best-effort
        save_main_config(named_path(name), copy);
        return true;
    } catch (const std::exception& e) {
        err = e.what();
        return false;
    }
}

bool SetupConfigStore::load_named(const std::string& name, std::string& err) {
    if (!valid_name(name)) {
        err = "invalid config name";
        return false;
    }
    const std::string path = named_path(name);
    std::error_code ec;
    if (!std::filesystem::exists(path, ec) || ec) {
        err = "no such named config: " + name;
        return false;
    }
    try {
        MainOptions loaded;  // start from defaults; overlay the named file
        load_main_config(path, loaded);
        {
            std::lock_guard<std::mutex> lk{mu_};
            draft_ = loaded;
        }
        return true;
    } catch (const std::exception& e) {
        err = e.what();
        return false;
    }
}

std::vector<std::string> SetupConfigStore::list_named() const {
    std::vector<std::string> names;
    std::error_code ec;
    if (!std::filesystem::exists(named_dir_, ec)) return names;
    for (const auto& entry :
         std::filesystem::directory_iterator(named_dir_, ec)) {
        if (ec) break;
        const auto& p = entry.path();
        if (p.extension() == ".yaml") names.push_back(p.stem().string());
    }
    std::sort(names.begin(), names.end());
    return names;
}

}  // namespace fitra::config

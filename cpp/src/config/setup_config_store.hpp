#pragma once
//
// SetupConfigStore — the editable config draft owned by the Setup module
// (docs/design/core-pipeline-setup-mode.md). Holds one MainOptions "draft"
// that the WebUI mutates (camera selection, engines, output targets), can be
// validated, written to the union config the daemon consumes, and saved /
// loaded under named slots (configs/named/<name>.yaml). Crow-free so it stays
// in fitra_config (no TensorRT/CUDA dependency); the JSON <-> MainOptions
// mapping lives in the web route layer.

#include <mutex>
#include <string>
#include <vector>

#include "config/main_config.hpp"

namespace fitra::config {

class SetupConfigStore {
public:
    // `seed` is the starting draft (the union config the daemon loaded, or
    // defaults). `union_path` is where write_union() persists for the daemon to
    // re-read; `named_dir` holds named slots.
    SetupConfigStore(MainOptions seed, std::string union_path,
                     std::string named_dir = "configs/named");

    MainOptions draft() const;                 // copy under lock
    void        set_draft(const MainOptions& opts);

    // Validate the draft as a union config (daemon form — does not require
    // cameras/engines, since intermediate setup states are legal). Returns
    // false + a user-facing reason in `err`.
    bool validate_draft(std::string& err) const;

    // Persist the draft to the union config path (atomic). The daemon re-reads
    // this on the next spawn.
    bool write_union(std::string& err);

    // Named-slot CRUD. `name` is sanitized to [A-Za-z0-9_-]; an invalid name
    // is rejected with a reason in `err`.
    bool save_named(const std::string& name, std::string& err);
    bool load_named(const std::string& name, std::string& err);  // into the draft
    std::vector<std::string> list_named() const;

    const std::string& union_path() const { return union_path_; }

private:
    static bool valid_name(const std::string& name);
    std::string named_path(const std::string& name) const;

    mutable std::mutex mu_;
    MainOptions draft_;
    std::string union_path_;
    std::string named_dir_;
};

}  // namespace fitra::config

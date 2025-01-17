/*
fat_esp32_filesystem.cpp - ESP3D fat filesystem configuration class

  Copyright (c) 2014 Luc Lebosse. All rights reserved.

  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with This code; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
// #define ESP_LOG_FEATURE LOG_OUTPUT_SERIAL0
#include "../../../include/esp3d_config.h"
#if (FILESYSTEM_FEATURE == ESP_FAT_FILESYSTEM)
#include <FS.h>

#include <stack>

#include "../esp_filesystem.h"
#include "FFat.h"

extern File tFile_handle[ESP_MAX_OPENHANDLE];

bool ESP_FileSystem::begin() {
  _started = FFat.begin();
  return _started;
}

void ESP_FileSystem::end() {
  FFat.end();
  _started = false;
}

size_t ESP_FileSystem::freeBytes() { return FFat.freeBytes(); }

size_t ESP_FileSystem::totalBytes() { return FFat.totalBytes(); }

size_t ESP_FileSystem::usedBytes() {
  return (FFat.totalBytes() - FFat.freeBytes());
}

uint ESP_FileSystem::maxPathLength() { return 32; }

bool ESP_FileSystem::rename(const char *oldpath, const char *newpath) {
  return FFat.rename(oldpath, newpath);
}

const char *ESP_FileSystem::FilesystemName() { return "FAT"; }

bool ESP_FileSystem::format() {
  bool res = FFat.format();
  if (res) {
    res = begin();
  }
  return res;
}

ESP_File ESP_FileSystem::open(const char *path, uint8_t mode) {
  esp3d_log("open %s as %s", path, (mode == ESP_FILE_WRITE ? "write" : "read"));
  // do some check
  if (((strcmp(path, "/") == 0) &&
       ((mode == ESP_FILE_WRITE) || (mode == ESP_FILE_APPEND))) ||
      (strlen(path) == 0)) {
    esp3d_log_e("reject  %s", path);
    return ESP_File();
  }
  // path must start by '/'
  if (path[0] != '/') {
    esp3d_log_e("%s is invalid path", path);
    return ESP_File();
  }
  File tmp = FFat.open(path, (mode == ESP_FILE_READ)    ? FILE_READ
                             : (mode == ESP_FILE_WRITE) ? FILE_WRITE
                                                        : FILE_APPEND);
  if (tmp) {
    ESP_File esptmp(&tmp, tmp.isDirectory(),
                    (mode == ESP_FILE_READ) ? false : true, path);
    esp3d_log("%s is a %s", path, tmp.isDirectory() ? "Dir" : "File");
    esp3d_log("path is %s and filename path is %s", path, tmp.path());
    return esptmp;
  } else {
    esp3d_log_e("open %s failed", path);
    return ESP_File();
  }
}

bool ESP_FileSystem::exists(const char *path) {
  bool res = false;
  // root should always be there if started
  if (strcmp(path, "/") == 0) {
    return _started;
  }
  res = FFat.exists(path);
  if (!res) {
    ESP_File root = ESP_FileSystem::open(path, ESP_FILE_READ);
    if (root) {
      res = root.isDirectory();
    }
    root.close();
  }
  return res;
}

bool ESP_FileSystem::remove(const char *path) { return FFat.remove(path); }

bool ESP_FileSystem::mkdir(const char *path) {
  String p = path;
  if (p[0] != '/') {
    p = "/" + p;
  }
  if (p[p.length() - 1] == '/') {
    if (p != "/") {
      p.remove(p.length() - 1);
    }
  }
  return FFat.mkdir(p);
}

bool ESP_FileSystem::rmdir(const char *path) {
  String p = path;
  if (!p.startsWith("/")) {
    p = '/' + p;
  }
  if (p != "/") {
    if (p.endsWith("/")) {
      p.remove(p.length() - 1);
    }
  }
  if (!exists(p.c_str())) {
    return false;
  }
  bool res = true;
  std::stack<String> pathlist;
  pathlist.push(p);
  while (pathlist.size() > 0 && res) {
    File dir = FFat.open(pathlist.top().c_str());
    File f = dir.openNextFile();
    bool candelete = true;
    while (f && res) {
      if (f.isDirectory()) {
        candelete = false;
        String newdir = pathlist.top() + '/';
        newdir += f.name();
        pathlist.push(newdir);
        f.close();
        f = File();
      } else {
        String filepath = pathlist.top() + '/';
        filepath += f.name();
        f.close();
        if (!FFat.remove(filepath.c_str())) {
          res = false;
        }
        f = dir.openNextFile();
      }
    }
    if (candelete) {
      if (pathlist.top() != "/") {
        res = FFat.rmdir(pathlist.top().c_str());
      }
      pathlist.pop();
    }
    dir.close();
  }
  p = String();
  esp3d_log("count %d", pathlist.size());
  return res;
}

void ESP_FileSystem::closeAll() {
  for (uint8_t i = 0; i < ESP_MAX_OPENHANDLE; i++) {
    tFile_handle[i].close();
    tFile_handle[i] = File();
  }
}

ESP_File::ESP_File(void *handle, bool isdir, bool iswritemode,
                   const char *path) {
  _isdir = isdir;
  _dirlist = "";
  _isfakedir = false;
  _index = -1;
  _filename = "";
  _name = "";
  _lastwrite = 0;
  _iswritemode = iswritemode;
  _size = 0;
  if (!handle) {
    esp3d_log("No handle");
    return;
  }
  bool set = false;
  for (uint8_t i = 0; (i < ESP_MAX_OPENHANDLE) && !set; i++) {
    if (!tFile_handle[i]) {
      tFile_handle[i] = *((File *)handle);
      // filename
      _filename = tFile_handle[i].path();
      // name
      if (_filename == "/") {
        _name = "/";
      } else {
        _name = tFile_handle[i].name();
        if (_name[0] == '/') {
          _name.remove(0, 1);
        }
        int pos = _name.lastIndexOf('/');
        if (pos != -1) {
          _name.remove(0, pos + 1);
        }
      }
      // size
      _size = tFile_handle[i].size();
      // time
      _lastwrite = tFile_handle[i].getLastWrite();
      _index = i;
      esp3d_log("Opening File at index %d", _index);
      esp3d_log("name: %s", _name.c_str());
      esp3d_log("filename: %s", _filename.c_str());
      esp3d_log("path: %s", tFile_handle[i].path());
      set = true;
    }
  }
  if (!set) {
    esp3d_log("No handle available");
  }
}

bool ESP_File::seek(uint32_t pos, uint8_t mode) {
  return tFile_handle[_index].seek(pos, (SeekMode)mode);
}

void ESP_File::close() {
  if (_index != -1) {
    esp3d_log("Closing File %s at index %d", _filename.c_str(), _index);
    esp3d_log("name: %s", _name.c_str());
    tFile_handle[_index].close();
    // reopen if mode = write
    // udate size + date
    if (_iswritemode && !_isdir) {
      esp3d_log("Updating %s size", _filename.c_str());
      File ftmp = FFat.open(_filename.c_str());
      if (ftmp) {
        _size = ftmp.size();
        esp3d_log("Size is %d", _size);
        _lastwrite = ftmp.getLastWrite();
        ftmp.close();
      }
    }
    tFile_handle[_index] = File();
    _index = -1;
  }
}

ESP_File ESP_File::openNextFile() {
  if ((_index == -1) || !_isdir) {
    esp3d_log("openNextFile %d failed", _index);
    return ESP_File();
  }
  File tmp = tFile_handle[_index].openNextFile();
  while (tmp) {
    esp3d_log("tmp name :%s %s", tmp.name(),
              (tmp.isDirectory()) ? "isDir" : "isFile");
    ESP_File esptmp(&tmp, tmp.isDirectory());
    esptmp.close();
    return esptmp;
  }
  return ESP_File();
}

#endif  // ESP_FAT_FILESYSTEM

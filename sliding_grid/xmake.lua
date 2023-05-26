add_rules("mode.debug", "mode.release")

set_languages("cxx11")

add_includedirs("/usr/local/include/eigen3")

add_requires("gtest", "glog") 

on_load(function (target)
    local include_dir = vformat("%s/include", os.scriptdir())
    target:add("includedirs", include_dir)
    local include_table = target:get("includedirs")
end)

add_defines("STAND_ALONE_BUILD")

add_requires("cmake::OpenCV", {alias = "opencv"})

-- target("sliding_grid_test")
--     set_kind("binary")
--     add_files("tests/test.cpp")
--     add_syslinks("pthread")
--     add_packages("gtest")
    -- add_packages("glog")

target("demo2d")
    set_kind("binary")
    add_files("demo/demo2d.cpp")
    add_packages("glog", "opencv")

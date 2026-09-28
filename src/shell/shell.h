#pragma once

namespace ind::shell {

// 初始化官方 Console 命令系统，但暂不启动 REPL。
// 可选模块应在 Initialize() 成功后、Start() 前注册自己的命令。
bool Initialize();

// 在已经初始化的 Hardware CDC 上启动 Console REPL。
bool Start();

} // namespace ind::shell

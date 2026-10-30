#pragma once

#include <stddef.h>
#include <esp_err.h>

#ifdef __cplusplus
#include <Console.h>

extern "C" {
#else
#include <esp_console.h>
#endif

// 节点和所有字符串必须具有静态生命周期，注册后不得修改或复制节点。
// name 不得为空或包含空白字符；help 是内置命令的保留名称。
// help、hint 可为 NULL；next 由框架维护，定义节点时填 NULL。
// func 的 argc 包含 argv[0] 的命令名；处理成功返回 0，失败返回非 0。
typedef struct IndShellCommand {
    const char *name;
    const char *help;
    const char *hint;
#ifdef __cplusplus
    ConsoleCommandFunc func;
#else
    esp_console_cmd_func_t func;
#endif
    struct IndShellCommand *next;
} IndShellCommand;

// 只收集节点，不访问 Console，也不分配堆内存。
// 必须在 shell::Initialize() 首次调用前串行注册，不支持并发或运行中注册。
// ESP_OK：注册成功，或同一节点已注册。
// ESP_ERR_INVALID_ARG：节点、名称或处理函数无效。
// ESP_ERR_INVALID_STATE：已关闭注册，或不同节点使用了重复/保留名称。
// 收集期间的第一个错误会保留，之后由 Initialize() 报错并返回 false。
// Initialize() 开始后调用只返回 ESP_ERR_INVALID_STATE，不改变已有命令表。
// 条件注册示例（C/C++ 均可）：
//   static IndShellCommand command = {"hello", "Print hello", NULL, HandleHello, NULL};
//   // 在模块初始化成功后：ind_shell_register_command(&command);
esp_err_t ind_shell_register_command(IndShellCommand *command);

#ifdef __cplusplus
} // extern "C"
#endif

// 无条件导出命令，在文件/命名空间作用域使用；C/C++ 均通过 GCC constructor 收集。
// alias 必须是当前翻译单元内唯一的 C 标识符，字符串化后作为命令名。
// handler 签名为 int (int argc, char **argv)；help_text、hint_text 使用字符串常量或 NULL。
// 构造函数只调用上面的注册入口，不依赖 Console、日志或其他全局对象。
// 示例：IND_SHELL_CMD_EXPORT_ALIAS(HandleHello, hello, "Print hello", NULL);
#define IND_SHELL_CMD_EXPORT_ALIAS(handler, alias, help_text, hint_text)                  \
    static IndShellCommand ind_shell_command_##alias = {                               \
        #alias, (help_text), (hint_text), (handler), NULL                                \
    };                                                                                 \
    static void ind_shell_export_##alias(void) __attribute__((constructor));            \
    static void ind_shell_export_##alias(void) {                                        \
        (void)ind_shell_register_command(&ind_shell_command_##alias);                   \
    }

// 以处理函数的标识符作为命令名；带命名空间的 C++ 函数使用上面的 ALIAS 版本。
#define IND_SHELL_CMD_EXPORT(handler, help_text, hint_text) \
    IND_SHELL_CMD_EXPORT_ALIAS(handler, handler, help_text, hint_text)

#ifdef __cplusplus
namespace ind::shell {

// 首次调用即关闭注册；初始化 Console 和 help，然后统一添加收集的命令。
// 成功后重复调用返回 true；Console 初始化/添加失败会清理 Console，可重试原表。
// 收集期间存在注册错误时返回 false，需修正注册代码并重启。暂不启动 REPL。
bool Initialize();

// 在已经初始化的 Hardware CDC 上启动 Console REPL。
bool Start();

} // namespace ind::shell
#endif

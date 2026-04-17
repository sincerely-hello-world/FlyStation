#!/usr/bin/env python3

def string_to_hex(s: str) -> str:
    """将字符串转为 '0xXX, 0xXX, ...' 格式的十六进制表示（逗号分隔）"""
    return ', '.join(f'0x{b:02X}' for b in s.encode('utf-8'))

def main():
    print("=== 字符串转十六进制工具 (输入 'exit' 退出) ===")
    while True:
        try:
            print("\n请输入一个字符串（输入 'exit' 退出）:")
            user_input = input().strip()

            if user_input.lower() == 'exit':
                print("再见！")
                break

            if not user_input:
                print("⚠️ 输入为空，请重新输入。")
                continue

            hex_str = string_to_hex(user_input)
            print("十六进制结果:")
            print(hex_str)

        except KeyboardInterrupt:
            print("\n\n收到 Ctrl+C，正在退出...")
            break
        except Exception as e:
            print(f"❌ 发生错误: {e}")

if __name__ == '__main__':
    main()
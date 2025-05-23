import whisper
import os

# ==== 1. 加载模型 ====
model = whisper.load_model("small")  # 可选 tiny, base, small, medium, large

# ==== 2. 输入音频文件路径 ====
audio_path = "output.wav"  # 替换为你的音频文件路径

# ==== 3. 执行转录（自动语言识别 + 分段） ====
print("开始转录...")
result = model.transcribe(audio_path, verbose=True)

# ==== 4. 打印整体文本 ====
print("\n=== 转录完成的全文 ===")
print(result["text"])

# ==== 5. 分段打印（含时间戳） ====
print("\n=== 含时间戳的分段文本 ===")
for segment in result["segments"]:
    print(f"[{segment['start']:.2f} - {segment['end']:.2f}]: {segment['text']}")

# ==== 6. 保存为 TXT 文件 ====
output_file = "transcription_result.txt"
with open(output_file, "w", encoding="utf-8") as f:
    f.write("完整转录文本：\n")
    f.write(result["text"] + "\n\n")
    f.write("分段内容（含时间戳）：\n")
    for segment in result["segments"]:
        f.write(f"[{segment['start']:.2f} - {segment['end']:.2f}]: {segment['text']}\n")

print(f"\n✅ 转录结果已保存至：{output_file}")


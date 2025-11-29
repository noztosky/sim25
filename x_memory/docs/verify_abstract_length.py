import re

with open('paper_xelatex02.tex', 'r', encoding='utf-8') as f:
    content = f.read()

# Look for the Korean abstract header
start_marker = r'{\centering{\fontsize{10}{12}\selectfont\bfseries 초\hspace{2em}록}\par}'
start_index = content.find(start_marker)

if start_index == -1:
    print("Korean Abstract start not found")
    exit()

# Find the content block
content_start_marker = r'{\fontsize{10}{12}\selectfont\ \ '
content_start = content.find(content_start_marker, start_index)

if content_start == -1:
    print("Korean Abstract content start not found")
    exit()

text_start = content_start + len(content_start_marker)
text_end = content.find(r'\par}', text_start)

korean_abstract_text = content[text_start:text_end]

print(f"New length (including spaces): {len(korean_abstract_text)}")
print(f"New length (excluding spaces): {len(korean_abstract_text.replace(' ', ''))}")
print(f"Content: {korean_abstract_text}")

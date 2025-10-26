#!/usr/bin/env python3
from __future__ import annotations

import ast
import tkinter as tk
from tkinter import ttk


ALLOWED_CHARS = "0123456789.+-*/()%^ xX÷×"


def sanitize(expr: str) -> str:
    # Normalize some symbols and keep only allowed characters
    expr = expr.replace("×", "*").replace("x", "*").replace("X", "*").replace("÷", "/")
    expr = expr.replace("^", "**")
    return "".join(ch for ch in expr if ch in ALLOWED_CHARS or ch == "*")


class SafeEvaluator:
    ALLOWED_BINOPS = (ast.Add, ast.Sub, ast.Mult, ast.Div, ast.Mod, ast.Pow)
    ALLOWED_UNARYOPS = (ast.UAdd, ast.USub)

    @classmethod
    def eval(cls, expr: str) -> float:
        try:
            node = ast.parse(expr, mode="eval")
        except SyntaxError as e:
            raise ValueError("Syntax error") from e
        return cls._eval_node(node.body)

    @classmethod
    def _eval_node(cls, node) -> float:
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
            return float(node.value)
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, cls.ALLOWED_UNARYOPS):
            val = cls._eval_node(node.operand)
            return +val if isinstance(node.op, ast.UAdd) else -val
        if isinstance(node, ast.BinOp) and isinstance(node.op, cls.ALLOWED_BINOPS):
            left = cls._eval_node(node.left)
            right = cls._eval_node(node.right)
            if isinstance(node.op, ast.Add):
                return left + right
            if isinstance(node.op, ast.Sub):
                return left - right
            if isinstance(node.op, ast.Mult):
                return left * right
            if isinstance(node.op, ast.Div):
                return left / right
            if isinstance(node.op, ast.Mod):
                return left % right
            if isinstance(node.op, ast.Pow):
                return left ** right
        if isinstance(node, ast.Expr):
            return cls._eval_node(node.value)
        raise ValueError("Unsupported expression")


class CalculatorApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Calculator")
        self.root.geometry("320x420")
        self.root.resizable(False, False)

        self.style = ttk.Style(self.root)
        try:
            self.style.theme_use("clam")
        except Exception:
            pass

        self.expr_var = tk.StringVar(value="")
        self.display = ttk.Entry(self.root, textvariable=self.expr_var, font=("Segoe UI", 18), justify="right")
        self.display.state(["readonly"])  # prevent direct typing into entry; we handle via bindings
        self.display.grid(row=0, column=0, columnspan=4, sticky="nsew", padx=8, pady=(10, 6))

        for i in range(4):
            self.root.grid_columnconfigure(i, weight=1, uniform="col")
        for i in range(1, 7):
            self.root.grid_rowconfigure(i, weight=1)

        btn = self._mk_button

        # Row 1: C, ⌫, (, )
        btn("C", 1, 0, self.clear, style="Accent.TButton")
        btn("⌫", 1, 1, self.backspace)
        btn("(", 1, 2, lambda: self.append("("))
        btn(")", 1, 3, lambda: self.append(")"))

        # Row 2: 7 8 9 /
        btn("7", 2, 0, lambda: self.append("7"))
        btn("8", 2, 1, lambda: self.append("8"))
        btn("9", 2, 2, lambda: self.append("9"))
        btn("÷", 2, 3, lambda: self.append("/"))

        # Row 3: 4 5 6 *
        btn("4", 3, 0, lambda: self.append("4"))
        btn("5", 3, 1, lambda: self.append("5"))
        btn("6", 3, 2, lambda: self.append("6"))
        btn("×", 3, 3, lambda: self.append("*"))

        # Row 4: 1 2 3 -
        btn("1", 4, 0, lambda: self.append("1"))
        btn("2", 4, 1, lambda: self.append("2"))
        btn("3", 4, 2, lambda: self.append("3"))
        btn("-", 4, 3, lambda: self.append("-"))

        # Row 5: 0 . = +
        btn("0", 5, 0, lambda: self.append("0"))
        btn(".", 5, 1, lambda: self.append("."))
        btn("=", 5, 2, self.equals, style="Accent.TButton")
        btn("+", 5, 3, lambda: self.append("+"))

        # Row 6: ^, %, CE, 1/x (optional functions)
        btn("x^y", 6, 0, lambda: self.append("**"))
        btn("%", 6, 1, lambda: self.append("%"))
        btn("CE", 6, 2, self.clear_entry)
        btn("1/x", 6, 3, self.reciprocal)

        # Keyboard bindings
        self.root.bind("<Key>", self._on_key)
        self.root.bind("<Return>", lambda e: self.equals())
        self.root.bind("<KP_Enter>", lambda e: self.equals())
        self.root.bind("<Escape>", lambda e: self.clear())
        self.root.bind("<Delete>", lambda e: self.clear())
        self.root.bind("<BackSpace>", lambda e: self.backspace())

    def _mk_button(self, text: str, r: int, c: int, cmd, style: str | None = None) -> None:
        opts = {"text": text, "command": cmd}
        if style:
            opts["style"] = style
        b = ttk.Button(self.root, **opts)
        b.grid(row=r, column=c, sticky="nsew", padx=6, pady=6, ipadx=4, ipady=10)

    def append(self, s: str) -> None:
        cur = self.expr_var.get()
        # Map caret to power token
        if s == "^":
            s = "**"
        self.expr_var.set(cur + s)

    def clear(self) -> None:
        self.expr_var.set("")

    def clear_entry(self) -> None:
        # Clear current entry (last token). If 'Error', clear all.
        expr = self.expr_var.get().rstrip()
        if not expr:
            return
        if expr == "Error":
            self.expr_var.set("")
            return
        # If ends with a number, remove that contiguous numeric token
        i = len(expr) - 1
        if expr[i].isdigit() or expr[i] == ".":
            while i >= 0 and (expr[i].isdigit() or expr[i] == "."):
                i -= 1
            self.expr_var.set(expr[: i + 1])
            return
        # Handle '**' as a unit (power)
        if expr.endswith("**"):
            self.expr_var.set(expr[:-2])
            return
        # If ends with operator or parenthesis, drop that one char
        if expr[-1] in "+-*/()%":
            self.expr_var.set(expr[:-1])
            return
        # Fallback: clear all
        self.expr_var.set("")

    def backspace(self) -> None:
        expr = self.expr_var.get()
        if not expr:
            return
        # handle '**' as a unit
        if expr.endswith("**"):
            self.expr_var.set(expr[:-2])
        else:
            self.expr_var.set(expr[:-1])

    def reciprocal(self) -> None:
        expr = self.expr_var.get().strip()
        if not expr:
            return
        self.expr_var.set(f"1/({expr})")

    def equals(self) -> None:
        raw = self.expr_var.get().strip()
        if not raw:
            return
        expr = sanitize(raw)
        try:
            result = SafeEvaluator.eval(expr)
        except Exception:
            self.expr_var.set("Error")
            return
        # Trim trailing .0
        s = ("%f" % result).rstrip("0").rstrip(".")
        self.expr_var.set(s)

    def _on_key(self, event) -> None:
        ch = event.char
        if ch in "\r\n":
            self.equals()
            return
        if ch == "\x1b":  # Esc
            self.clear()
            return
        if ch == "\x08":  # Backspace
            self.backspace()
            return
        if not ch:
            return
        if ch in ALLOWED_CHARS:
            if ch == "^":
                self.append("**")
            elif ch in ("×", "x", "X"):
                self.append("*")
            elif ch == "÷":
                self.append("/")
            else:
                self.append(ch)


def main() -> None:
    root = tk.Tk()
    app = CalculatorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()



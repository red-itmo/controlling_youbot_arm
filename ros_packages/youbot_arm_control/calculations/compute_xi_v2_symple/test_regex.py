import re


def python_gencode(expr):
    """ python code cleaner"""

    def replaceDotsQ(m):
        "replace Derivativ and replace q_i(t) on dq_i and next dq[i]"
        lala = re.search(r"[d]*q_[1-5]", m.group(0))
        t = re.findall(r"[t]+", m.group(0))
        num = re.search(r"[1-5]", lala.group(0))
        n = int(num.group(0))-1
        if len(t) - 2 == 1:
            return "dq[" + str(n) + "]"
        elif len(t) - 2 == 2:
            return "ddq[" + str(n) + "]"

    def replaceADi(m):
        paramName = re.search(r"[ad]", m.group(0))
        num = re.search(r"[1-5]", m.group(0))
        n = int(num.group(0))-1
        return str(paramName.group(0)) + "[" + str(n) + "]"

    def deleteT(m):
        return ""

    def replaceQi(m):
        q_i = re.search(r"q_[1-5]", m.group(0))
        num = re.search(r"[1-5]", m.group(0))
        n = int(num.group(0))-1
        return "q[" + str(n) + "]"

    # replace all Derivative(.*)
    replace_dotsq = re.sub(r"Derivative\([d]*q_[1-5]+\(t\),[\w ,]+\)", replaceDotsQ, expr)
    # delete all "(t)"
    replace_t = re.sub(r"\(t\)", deleteT, replace_dotsq)
    # replace all q_i to q[i]
    replace_qi = re.sub(r"q_[1-5]", replaceQi, replace_t)
    # replace DH parameters
    replace_adi = re.sub(r"[ad]_[1-5]", replaceADi, replace_qi)
    return replace_adi


if __name__ == "__main__":
    e = "a_1 * a_2 * a_3 * d_1 * d_5 * Derivative(q_3(t), t) * Derivative(q_1(t), t, t) * Derivative(q_4(t), t^2)"
    okS = python_gencode(e)
    print(okS)

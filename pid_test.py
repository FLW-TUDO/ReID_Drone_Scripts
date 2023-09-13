from simple_pid import PID

POS = 50

iter = 0 
iter_break = 0
pid = PID(0.01,0,0,0)

while abs(POS) > 0.0001:
    iter += 1
    prev_POS = POS
    POS = POS + pid(POS)
    print(f"neue Position: {POS} nach {iter} Iterationen")
    if round(POS,5) == round(prev_POS,5):
        iter_break += 1
    else:
        iter_break = 0

    if iter_break >= 250:
        break
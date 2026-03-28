from gss6100 import GSS6100

g = GSS6100()

for q in ["*IDN?", "LEVL ?", "SNUM ?", "STAT ?", "TIOP ?"]:
    try:
        print(f"{q:<8} -> {g.query(q)}")
    except Exception as e:
        print(f"{q:<8} -> ERROR: {e}")


print("ID:    ", g.identify())
print("State: ", g.query("STAT ?"))
print("Level: ", g.query("LEVL ?"))

g.write("LEVL -2.0")

print("Level: ", g.query("LEVL ?"))
print("Error: ", g.query("SERR ?"))

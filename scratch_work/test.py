from hardware_comms import stellarnet_peter as snp
import matplotlib.pyplot as plt
import clipboard_and_style_sheet

clipboard_and_style_sheet.style_sheet()

spec = snp.Spectrometer()
plt.figure()
plt.plot(spec.wl_nm, spec.spectrum()[1])

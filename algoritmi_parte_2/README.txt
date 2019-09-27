Breve descrizione degli algoritmi presenti in daVinci/src:

1) segmentation:
L'algoritmo si mette in ascolto sul topic su cui viene pubblicata la scena.
Vengono applicati sia il color segmentation (HSV) che il cluster segmentation.
Possibilità di visualizzare gli oggetti segmentati su topic o salvarli in un file .pcd

2) color_segmentation:
Modifica del segmentation in cui viene applicata solo la segmentazione in base
al colore (HSV)

3) correspondence_ROS:
Versione modificata del correspondence_grouping, l'algoritmo si mette in ascolto sul topic
su cui viene pubblicata la scena e prende come input a riga di comando una serie di modelli.
Controlla se all'interno della scena sono presenti quei modelli e mette in mostra le varie
corrispondenze.

4) region_growing_ROS:
Versione del region_growing_ROS adattata a ROS

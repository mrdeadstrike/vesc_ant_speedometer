# --- Email-настройки ---
SMTP_SERVER = "smtp.mail.ru"
SMTP_PORT = 465
SENDER_EMAIL = "tagmeet@mail.ru"
SENDER_PASSWORD = "xJhYSdv448js1Detwcsc"
RECEIVER_EMAIL = "ilua.pisar@gmail.com"
FILENAME = "/home/dead/trip.mp4"

from email.message import EmailMessage
import smtplib
import ssl

def send_email_with_attachment():
  msg = EmailMessage()
  msg['Subject'] = 'Видео с Raspberry Pi'
  msg['From'] = SENDER_EMAIL
  msg['To'] = RECEIVER_EMAIL
  msg.set_content('Вот видеофайл с записи экрана.')

  with open(FILENAME, 'rb') as f:
    file_data = f.read()
    msg.add_attachment(file_data, maintype='video', subtype='mp4', filename=FILENAME)

  context = ssl.create_default_context()
  with smtplib.SMTP(SMTP_SERVER, SMTP_PORT) as smtp:
    smtp.starttls(context=context)
    smtp.login(SENDER_EMAIL, SENDER_PASSWORD)
    smtp.send_message(msg)

  print(">>> Файл отправлен на email.")

send_email_with_attachment()
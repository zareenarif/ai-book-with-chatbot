from sendgrid import SendGridAPIClient
from sendgrid.helpers.mail import Mail
from src.config import settings
from typing import Optional


def send_verification_email(email: str, token: str) -> bool:
    """Send email verification link"""
    verification_url = f"http://localhost:3000/verify?token={token}"

    message = Mail(
        from_email=settings.SENDGRID_FROM_EMAIL,
        to_emails=email,
        subject=f'{settings.SENDGRID_FROM_NAME} - Verify Your Email',
        html_content=f'''
        <h2>Welcome to {settings.SENDGRID_FROM_NAME}!</h2>
        <p>Please verify your email address by clicking the link below:</p>
        <p><a href="{verification_url}">Verify Email</a></p>
        <p>If you didn't create an account, you can safely ignore this email.</p>
        '''
    )

    try:
        sg = SendGridAPIClient(settings.SENDGRID_API_KEY)
        sg.send(message)
        return True
    except Exception as e:
        print(f"Error sending verification email: {str(e)}")
        return False


def send_password_reset_email(email: str, token: str) -> bool:
    """Send password reset link"""
    reset_url = f"http://localhost:3000/reset-password?token={token}"

    message = Mail(
        from_email=settings.SENDGRID_FROM_EMAIL,
        to_emails=email,
        subject=f'{settings.SENDGRID_FROM_NAME} - Password Reset',
        html_content=f'''
        <h2>Password Reset Request</h2>
        <p>Click the link below to reset your password:</p>
        <p><a href="{reset_url}">Reset Password</a></p>
        <p>This link will expire in 1 hour.</p>
        <p>If you didn't request this, you can safely ignore this email.</p>
        '''
    )

    try:
        sg = SendGridAPIClient(settings.SENDGRID_API_KEY)
        sg.send(message)
        return True
    except Exception as e:
        print(f"Error sending password reset email: {str(e)}")
        return False

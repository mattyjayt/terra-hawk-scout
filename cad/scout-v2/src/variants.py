"""One @model per Scout v2 variant (generated list; specs in scout_v2.VARIANTS).

lidar {ld19, c1} x motor {tt_stock, hiwonder, fit0450, yahboom} x payload {pay, nopay}, all with the MDD3A,
plus scout_v2_tb6612 (default variant with a Pololu TB6612 carrier instead of the MDD3A).
"""
from cadgen import model

from scout_v2 import VARIANTS, scout


@model
def scout_v2__ld19_tt_stock_pay():
    return scout(*VARIANTS["scout_v2__ld19_tt_stock_pay"], label="scout_v2__ld19_tt_stock_pay")


@model
def scout_v2__ld19_tt_stock_nopay():
    return scout(*VARIANTS["scout_v2__ld19_tt_stock_nopay"], label="scout_v2__ld19_tt_stock_nopay")


@model
def scout_v2__ld19_hiwonder_pay():
    return scout(*VARIANTS["scout_v2__ld19_hiwonder_pay"], label="scout_v2__ld19_hiwonder_pay")


@model
def scout_v2__ld19_hiwonder_nopay():
    return scout(*VARIANTS["scout_v2__ld19_hiwonder_nopay"], label="scout_v2__ld19_hiwonder_nopay")


@model
def scout_v2__ld19_fit0450_pay():
    return scout(*VARIANTS["scout_v2__ld19_fit0450_pay"], label="scout_v2__ld19_fit0450_pay")


@model
def scout_v2__ld19_fit0450_nopay():
    return scout(*VARIANTS["scout_v2__ld19_fit0450_nopay"], label="scout_v2__ld19_fit0450_nopay")


@model
def scout_v2__ld19_yahboom_pay():
    return scout(*VARIANTS["scout_v2__ld19_yahboom_pay"], label="scout_v2__ld19_yahboom_pay")


@model
def scout_v2__ld19_yahboom_nopay():
    return scout(*VARIANTS["scout_v2__ld19_yahboom_nopay"], label="scout_v2__ld19_yahboom_nopay")


@model
def scout_v2__c1_tt_stock_pay():
    return scout(*VARIANTS["scout_v2__c1_tt_stock_pay"], label="scout_v2__c1_tt_stock_pay")


@model
def scout_v2__c1_tt_stock_nopay():
    return scout(*VARIANTS["scout_v2__c1_tt_stock_nopay"], label="scout_v2__c1_tt_stock_nopay")


@model
def scout_v2__c1_hiwonder_pay():
    return scout(*VARIANTS["scout_v2__c1_hiwonder_pay"], label="scout_v2__c1_hiwonder_pay")


@model
def scout_v2__c1_hiwonder_nopay():
    return scout(*VARIANTS["scout_v2__c1_hiwonder_nopay"], label="scout_v2__c1_hiwonder_nopay")


@model
def scout_v2__c1_fit0450_pay():
    return scout(*VARIANTS["scout_v2__c1_fit0450_pay"], label="scout_v2__c1_fit0450_pay")


@model
def scout_v2__c1_fit0450_nopay():
    return scout(*VARIANTS["scout_v2__c1_fit0450_nopay"], label="scout_v2__c1_fit0450_nopay")


@model
def scout_v2__c1_yahboom_pay():
    return scout(*VARIANTS["scout_v2__c1_yahboom_pay"], label="scout_v2__c1_yahboom_pay")


@model
def scout_v2__c1_yahboom_nopay():
    return scout(*VARIANTS["scout_v2__c1_yahboom_nopay"], label="scout_v2__c1_yahboom_nopay")


@model
def scout_v2_tb6612():
    return scout(*VARIANTS["scout_v2_tb6612"], label="scout_v2_tb6612")


MODELS = {f.__name__: f for f in (scout_v2__ld19_tt_stock_pay, scout_v2__ld19_tt_stock_nopay, scout_v2__ld19_hiwonder_pay, scout_v2__ld19_hiwonder_nopay, scout_v2__ld19_fit0450_pay, scout_v2__ld19_fit0450_nopay, scout_v2__ld19_yahboom_pay, scout_v2__ld19_yahboom_nopay, scout_v2__c1_tt_stock_pay, scout_v2__c1_tt_stock_nopay, scout_v2__c1_hiwonder_pay, scout_v2__c1_hiwonder_nopay, scout_v2__c1_fit0450_pay, scout_v2__c1_fit0450_nopay, scout_v2__c1_yahboom_pay, scout_v2__c1_yahboom_nopay, scout_v2_tb6612)}


if __name__ == "__main__":
    for _f in MODELS.values():
        _f()

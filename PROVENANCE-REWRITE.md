# Commit-hash provenance: the 2026-09-11 single-author rewrite

On 2026-09-11 the full history of this repository was re-authored to a single author
(`DeanDonkov <donkov.business@gmail.com>`) before first publication. Every commit therefore has a new SHA.

**No file content changed.** The rewrite touched only the author and committer identity fields. This is
mechanically verifiable: the tree object of every rewritten commit is byte-identical to the tree of the commit it
replaces, checked for all 41 commits at rewrite time (0 mismatches).

This matters because the archived validation outputs in `validation/` record the code commit they were produced
from, and the accompanying manuscript cites those commits as the provenance of its measured ([M]) results. Those
records were **not** edited — they are evidence of what was run, and rewriting them would falsify them. Instead,
every pre-rewrite hash they cite maps to a post-rewrite hash in the table below, and the pre-rewrite history is
retained offline by the author as `crtk-conformance-rc5.bundle`.

Commits referenced by the archives and the changelog:

| cited as | version / archive | post-rewrite |
|---|---|---|
| `1c06238` | v0.1.3 code; `validation/v0.1.3/` | `004ef30` |
| `d322518` | v0.1.2 campaign; `validation/v0.1.2/` | `46fe8de` |
| `f9e2b51` | v0.1.2 superseded runs | `e805132` |
| `e546af1` | v0.1.1 campaign; `validation/v0.1.1/` | `44cd1f1` |
| `5393272` | v0.1.0 archive | `2aa45b1` |
| `b074e2e` | tag `v0.1.0` | `0a37850` |
| `7e0300c` | v0.1.4 (the rewrite's own parent state) | `510ca05` |

Full map, one commit per line, `pre-rewrite<TAB>post-rewrite`:

```
01750a446d43743bc6592ff3d56a15979a0090e0	9194103c1a61f7ce0510722d01759585b9c3365f
053bcd3be9246427c749fef8bddab70aeac7f3b2	cf6b51c44800559792bdf8ecb247f3260de7eb55
0f4549bca1aa1a8b1b23decce5a837bb35098d89	6f3be15f2db9b154aecbd63e1108f662706d2183
1a1a0304f0854bd42b33ec1cab2ee937d7257450	2088f251913c55e911e01daf7649c8b682d1244c
1c06238b8bbabc09a6b969eb1d9833660bd2e11c	004ef30ae471eb46f70f348f2c4db561117c4bdf
2cc93219a314815795276bf8039dcb15dfcc2874	9c0d3e948a9bfe7acd9587d11d62c2d7b4ea85e5
40b3d4575cef6460075cec76666db79b66139f12	9195e2207eb1ec46cfac7bc6542fe2f069d4b8be
4741cf4f7a2f2b589808ff92e3426642826dae51	e77651b277054da222144558b4a244ed55e735f5
4e16165bc1b664d7f438eb7fbd9b63d634a7090c	ed726dfd6caa5a4bda0dbfa98c37a7c311d286a7
51c194b7948f258c503d61031cd99462fc728014	c300f286c1586ec59b3eb9d02547b369ad2ca180
5393272c3e17b765eb1613ac57fa79789fb5c814	2aa45b1a5bb5704848a2a55fa24f8b8d405442e0
57093d2ba37ac06b46c6d5da52fe557637f0dff2	1ebc8fd365acfc46584ae662941da2adad6a09f3
59c48763c99cc1ae20daf65e33e042eb480c7bb3	c288c575d3ed7187f53c9d123d414d6f1196d39c
63501b8cfa9b0e62a0f66452df07daa99d80d363	270a769ed63c55830049d68e9f7312b20568e0ae
679db68ea8889a456a15e306c0eb50afd7326b78	c36b8a1c9ec6d1c8b30b8f1933cfcfca0d913eb5
6b796e87d7664644a1c70df7f62305d316d7fb32	ac2317540d0b2f37d76e0ce82654f08c9de5778b
6e93bd1a2552b30779726deab3b518f746151e81	e8a9f5e06722e62d5f012559c6130072329ab374
7290941e0820b0865a721efecd58ea038b4900ff	49a4fa635f6cda1a9764a6aea9b28001ccc9a239
77b64f62cb0c45a2dd78056b45442710a9b81a7a	08709f34d206a2820ec993a38dc29e0ed166613e
7c4ad1d832aa69c630c4bb00c3911855dd91fd63	7d3f90888a6e74b3b30fd2ea2d563494dbc51425
7e0300c958d1a3797d931f515243744e356839a9	510ca05d34bf11d4300571c59ca5987d8579cd8d
804cf388f2a813e5fe01b8b49bda0a70901e4fec	d88606fc247dcceb36aed3c765155cfeddad5bff
806ccd9d08e13195742c9ab43333400563f53bec	4ec9d5c8b4c917fad8b99e7bd4218eb5b64c5ae6
8267316506e5ea5537db810129daf17077f15a34	fb65f32740fae7acd30bdc6ea8b9f2a3e9b8620e
8a965ea543d2ac6c5de28481ae3e30184265a24f	0b982ae9ad6e0b275ef441beff8b7317874bf6d0
9710a2b9f5df0b22030ecd2546a9bf4dd2350b3e	2c4d67dfe7c7f9dc2bfc0168b3a91cde0a159ece
a5cb73e9c7c1ab9b45e77198f1b127de1c24292c	8b075ec4e4bc0ac00a64b5fddd82745fd711f7ba
ab465e0c477027d7ae4e79eb81c368ab642b9d05	fb1de3de993b1c6666bef28f74794ed9c5b928f8
ad78e5c12758844a0a407e2841f9e7468fb98690	e727914eb7769f73e6dbceb948695debb46a8912
aff6da573749d0d78357fc74885c56065514c941	f0a1ff9fa93f6720d266ab6a1f3b757d4b3e6646
b074e2e692b5619f4fad0bdb0c088fcbceb9547c	0a3785081cbc56f8e05eb55928f6aca8a2e63b6a
c19bd6e83c9eec510920e857e26ad71b00b3c920	c27c6b915c0b1a526a65eac94605d39898f2561e
c1b4f71ac4bca09ac813043c1817455a9da6e0ea	5f245901a308777b722e4048c1b56e747afec0ba
c3ecdc18e4e215c3191129211c96dc2ce4ab67e5	d0a3d5c27ccf9ffaf2f2cd0168a7490cd37d89d5
d322518f8fbd0d00d68b631e3a4419337f4947ed	46fe8de560b5a9ea872cef80a1b142581c0ef156
d8ef24cb6de340e8c949abb549b588e941f518ad	e8a1f5dfdcc7aa5463feb7074d52441f51300a80
e546af16297ce5c83a7a916b00e4b3c7ac712c62	44cd1f1162e7b8edf892831b9fddc5d4c56a5c92
ef65ff637abb31e39f12ffda25ed05553afc9d13	5f5e6d097890f14819eeec5deee473f588836987
f8a09ff8796730e48891658a7324a5c14c11d675	35c7345239c9dd8c410a72e0893cd17d9abd1a48
f9e2b51d1908f7987876baa27a26af2711f13cd8	e805132d1fd87d488a4602d27568599b37d8e671
fb5eebd28da314c0e4368310078be51289e5742d	e82f8160f6cca7f49b5170d4b35064d2df8fa7c8
```
